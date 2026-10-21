"""Fly the corridor on the forward lap: turn along the corridor, then run straight to the delivery zone.

Action: /navigate_corridor (mission_interfaces/NavigateCorridor, no goal fields; everything is a ROS parameter)
Input:  /fmu/out/vehicle_local_position (px4_msgs/VehicleLocalPosition)
Input:  /perception/obstacles (mission_interfaces/Obstacles), unused on the forward lap (no obstacles)
Output: /flight_controller/cmd_vel_body (geometry_msgs/Twist, body FLU)

Steps of one goal:
  1. Yaw: turn on the spot (holding position and height) to the corridor heading.
  2. Run: fly a fixed straight line from the pose after the turn, holding the line (cross-track),
     the height and the heading, with a ramped speed profile that brakes onto the stop distance.
  3. Stop at the delivery zone: after `entrance_distance + corridor_length + zone_margin` metres, or
     `zone_margin` metres after entering the zone read from `zone_file`, whichever comes first
     (`stop_mode`: distance | geofence | either).
Parameters are read at the start of every goal, so `ros2 param set` / `ros2 param load` on the field
take effect without restarting the node.

Corridor heading (`heading_source`):
  param    corridor_heading_deg, PX4 heading in degrees, 0 = north, clockwise positive (default)
  zone     bearing from the drone to the centre of the zone in `zone_file`
  current  skip the yaw step and fly along the current heading
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from mission_interfaces.action import NavigateCorridor
from mission_interfaces.msg import Obstacles
from px4_msgs.msg import VehicleLocalPosition
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from corridor_navigator import corridor_logic as cl
from flight_controller.px4_utils import PX4_QOS, px4_topic

CANCELED = object()  # sentinel returned by the steps when the goal was cancelled

GEOFENCE_TRUST_BEFORE_EXIT = 1.5  # m; the zone is only trusted this close to the corridor exit or later
STALE_POSE_TIMEOUT = 0.5  # s without a PX4 local position -> abort
ZONE_MAX_RANGE = 500.0  # m; a zone further than this from the drone is a bad file (swapped lat/lon?)
YAW_STABLE_TIME = 1.0  # s inside the yaw tolerance before the run starts
YAW_FALLBACK = 10.0  # deg; after the yaw timeout, carry on if at least this close
SETTLE_TIMEOUT = 3.0  # s to come to rest at the end
SETTLE_SPEED = 0.15  # m/s

PARAMS = {
    # distances and stop rule
    "entrance_distance": 2.5,  # m from the pose after alignment to the corridor entrance (about 2-3 m)
    "corridor_length": 10.0,  # m
    "zone_margin": 1.0,  # m to go into the zone before stopping ("a short distance in")
    "stop_mode": "either",  # distance | geofence | either
    "zone_file": "",  # text file, one "lat,lon" per line; empty = no zone
    "max_overrun": 5.0,  # m beyond the distance target allowed in geofence-only mode
    "goal_tolerance": 0.15,  # m
    # heading
    "heading_source": "param",  # param | zone | current
    "corridor_heading_deg": float("nan"),  # PX4 heading, 0 = north, clockwise
    "yaw_tolerance_deg": 3.0,
    "yaw_timeout": 15.0,  # s
    # speed
    "cruise_speed": 0.8,  # m/s
    "max_accel": 0.5,  # m/s^2
    # safety
    "max_cross_track": 1.0,  # m from the line before aborting (the corridor is 3.5 m wide)
    "expected_altitude": 3.0,  # m, only used to warn
    "loop_rate": 20.0,  # Hz; flight_controller holds position if commands stop for 0.5 s
}
GAIN_NAMES = ("cross_kp", "max_lateral", "alt_kp", "max_vertical", "yaw_kp", "max_yaw_rate", "hold_kp", "max_hold")


class CorridorNavigator(Node):
    def __init__(self):
        super().__init__("corridor_navigator")
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        for name in GAIN_NAMES:
            self.declare_parameter(name, getattr(cl.Gains(), name))

        self.obstacles = None  # latest Obstacles; not used on the forward lap
        self.local_msg = None  # latest VehicleLocalPosition
        self.local_time = 0.0  # time.monotonic() of local_msg
        self.busy = False

        group = ReentrantCallbackGroup()
        self.create_subscription(Obstacles, "/perception/obstacles", self.obstacles_cb, 10, callback_group=group)
        self.create_subscription(
            VehicleLocalPosition, px4_topic("/fmu/out/vehicle_local_position", VehicleLocalPosition),
            self.local_pos_cb, PX4_QOS, callback_group=group,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/flight_controller/cmd_vel_body", 10)
        ActionServer(
            self, NavigateCorridor, "/navigate_corridor", self.execute,
            goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )
        self.get_logger().info("corridor_navigator ready (forward lap, straight run, no obstacle avoidance).")

    # ---------------- callbacks and helpers ----------------

    def obstacles_cb(self, msg: Obstacles):
        self.obstacles = msg

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_msg = msg
        self.local_time = time.monotonic()

    def on_goal(self, _goal):
        if self.busy:
            self.get_logger().warn("Rejecting NavigateCorridor: one is already running.")
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    def current_pose(self):
        """Latest pose, or None if PX4 data is missing, stale or not good enough to fly on."""
        msg = self.local_msg
        if msg is None or time.monotonic() - self.local_time > STALE_POSE_TIMEOUT:
            return None
        if not (msg.xy_valid and msg.z_valid and msg.heading_good_for_control):
            return None
        return cl.Pose(msg.x, msg.y, msg.z, msg.heading, math.hypot(msg.vx, msg.vy))

    def publish(self, forward=0.0, left=0.0, up=0.0, yaw_rate=0.0):
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z = float(forward), float(left), float(up)
        cmd.angular.z = float(yaw_rate)
        self.cmd_vel_pub.publish(cmd)

    def load_zone(self, path):
        """Zone corners as local (north, east) from the text file; raises ValueError with the reason."""
        if not path:
            raise ValueError("no zone_file set")
        if not os.path.isfile(path):
            raise ValueError(f"zone_file '{path}' not found")
        msg = self.local_msg
        if msg is None or not msg.xy_global:
            raise ValueError("PX4 has no global reference for the local position yet")
        corners = [cl.latlon_to_ne(lat, lon, msg.ref_lat, msg.ref_lon) for lat, lon in cl.parse_zone_file(path)]
        centre = cl.centroid(corners)
        distance = math.hypot(centre[0] - msg.x, centre[1] - msg.y)
        if distance > ZONE_MAX_RANGE:
            raise ValueError(f"zone centre is {distance:.0f} m from the drone; check the file (lat,lon order?)")
        return corners

    # ---------------- action ----------------

    def execute(self, goal_handle):
        result = NavigateCorridor.Result()
        error = None
        try:
            error = self.fly_corridor(goal_handle)
        except Exception as exc:  # never leave the drone with a velocity command running
            self.get_logger().error(f"NavigateCorridor crashed: {exc!r}")
            error = f"internal error: {exc!r}"
        finally:
            self.publish()  # zero velocity; flight_controller then holds position
            self.busy = False

        if error is None:
            result.success, result.message = True, "corridor flown, stopped in the delivery zone"
            goal_handle.succeed()
        elif error is CANCELED:
            result.success, result.message = False, "canceled"
            goal_handle.canceled()
        else:
            result.success, result.message = False, error
            self.get_logger().error(f"NavigateCorridor: {error}")
            goal_handle.abort()
        return result

    def fly_corridor(self, goal_handle):
        """Returns None on success, CANCELED, or an error string."""
        cfg = {name: self.get_parameter(name).value for name in PARAMS}
        gains = cl.Gains(**{name: float(self.get_parameter(name).value) for name in GAIN_NAMES})
        dt = 1.0 / cfg["loop_rate"]
        mode = cfg["stop_mode"]
        if mode not in ("distance", "geofence", "either"):
            return f"stop_mode must be distance, geofence or either, not '{mode}'"
        if cfg["cruise_speed"] <= 0.0 or cfg["max_accel"] <= 0.0:
            return "cruise_speed and max_accel must be > 0"

        pose = self.current_pose()
        if pose is None:
            return "no valid PX4 local position"
        z_ref = pose.down  # hold the height reached by the descent before this step
        if abs(-z_ref - cfg["expected_altitude"]) > 0.7:
            self.get_logger().warn(f"Height is {-z_ref:.1f} m, expected about {cfg['expected_altitude']:.1f} m.")

        zone = None
        if mode in ("geofence", "either") or cfg["heading_source"] == "zone":
            try:
                zone = self.load_zone(cfg["zone_file"])
            except (ValueError, OSError) as exc:
                if mode == "geofence" or cfg["heading_source"] == "zone":
                    return f"zone needed but unusable: {exc}"
                self.get_logger().warn(f"No usable zone ({exc}); stopping on distance only.")
                mode = "distance"

        theta, error = self.corridor_heading(cfg, pose, zone)
        if error:
            return error
        self.get_logger().info(
            f"NavigateCorridor: heading {math.degrees(theta):.1f} deg (now {math.degrees(pose.heading):.1f}), "
            f"entrance {cfg['entrance_distance']:.1f} m + corridor {cfg['corridor_length']:.1f} m "
            f"+ margin {cfg['zone_margin']:.1f} m, mode {mode}, cruise {cfg['cruise_speed']:.2f} m/s."
        )

        error = self.turn_to(goal_handle, theta, z_ref, cfg, gains, dt)
        if error:
            return error
        return self.run_line(goal_handle, theta, z_ref, zone, mode, cfg, gains, dt)

    def corridor_heading(self, cfg, pose, zone):
        """(heading rad, None) or (None, error)."""
        source = cfg["heading_source"]
        if source == "current":
            return pose.heading, None
        if source == "param":
            degrees = cfg["corridor_heading_deg"]
            if not math.isfinite(degrees):
                return None, ("corridor_heading_deg is not set (set it, or heading_source:=current "
                              "to fly along the current heading)")
            return cl.wrap_pi(math.radians(degrees)), None
        if source == "zone":
            north, east = cl.centroid(zone)
            return math.atan2(east - pose.east, north - pose.north), None
        return None, f"heading_source must be param, zone or current, not '{source}'"

    # ---------------- step 1: turn along the corridor ----------------

    def turn_to(self, goal_handle, theta, z_ref, cfg, gains, dt):
        tolerance = math.radians(cfg["yaw_tolerance_deg"])
        pose = self.current_pose()
        if pose is None:
            return "lost PX4 local position"
        hold = (pose.north, pose.east)
        started = time.monotonic()
        stable_since = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return CANCELED
            pose = self.current_pose()
            if pose is None:
                return "lost PX4 local position while turning"
            error = cl.wrap_pi(theta - pose.heading)
            if abs(error) < tolerance:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= YAW_STABLE_TIME:
                    self.get_logger().info(f"Aligned with the corridor (yaw error {math.degrees(error):+.1f} deg).")
                    return None
            else:
                stable_since = None
            if time.monotonic() - started > cfg["yaw_timeout"]:
                if abs(error) < math.radians(YAW_FALLBACK):
                    self.get_logger().warn(f"Yaw timeout, {math.degrees(error):+.1f} deg off: carrying on.")
                    return None
                return f"could not align: still {math.degrees(error):+.1f} deg off after {cfg['yaw_timeout']:.0f} s"
            self.publish(*cl.turn_in_place_command(hold[0], hold[1], theta, pose, z_ref, gains))
            self.get_logger().info(f"  turning, yaw error {math.degrees(error):+.1f} deg", throttle_duration_sec=1.0)
            time.sleep(dt)
        return "node shutting down"

    # ---------------- step 2 and 3: run along the line and stop ----------------

    def run_line(self, goal_handle, theta, z_ref, zone, mode, cfg, gains, dt):
        pose = self.current_pose()
        if pose is None:
            return "lost PX4 local position"
        line = cl.Line(pose.north, pose.east, theta)
        distance_target = cfg["entrance_distance"] + cfg["corridor_length"] + cfg["zone_margin"]
        trust_from = max(0.0, cfg["entrance_distance"] + cfg["corridor_length"] - GEOFENCE_TRUST_BEFORE_EXIT)

        if zone is not None and cl.point_in_polygon(pose.north, pose.east, zone):
            if mode == "geofence":
                return "the drone starts inside the zone: wrong zone file?"
            self.get_logger().error("The drone starts inside the zone: ignoring the zone, distance only.")
            zone, mode = None, "distance"

        timeout = 2.0 * (distance_target + cfg["max_overrun"]) / cfg["cruise_speed"] + 10.0
        started = time.monotonic()
        enter_s = None
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                return CANCELED
            pose = self.current_pose()
            if pose is None:
                return "lost PX4 local position while flying the corridor"
            elapsed = time.monotonic() - started
            along, right = line.project(pose.north, pose.east)
            if abs(right) > cfg["max_cross_track"]:
                return f"cross-track error {right:+.2f} m exceeds {cfg['max_cross_track']:.2f} m at {along:.1f} m"
            if elapsed > timeout:
                return f"timeout after {elapsed:.0f} s at {along:.1f} m"

            if zone is not None and enter_s is None and along >= trust_from:
                if cl.point_in_polygon(pose.north, pose.east, zone):
                    enter_s = along
                    self.get_logger().info(f"Entered the delivery zone at {along:.1f} m.")
            target = cl.stop_target(mode, distance_target, enter_s, cfg["zone_margin"], cfg["max_overrun"])
            remaining = target - along
            if remaining <= cfg["goal_tolerance"]:
                break

            speed = cl.speed_profile(elapsed, remaining, cfg["cruise_speed"], cfg["max_accel"])
            self.publish(*cl.run_command(line, pose, speed, z_ref, gains))
            self.get_logger().info(
                f"  {along:5.1f}/{target:.1f} m  cross {right:+.2f} m  speed {speed:.2f} m/s  "
                f"yaw err {math.degrees(cl.wrap_pi(theta - pose.heading)):+.1f} deg",
                throttle_duration_sec=1.0,
            )
            time.sleep(dt)
        else:
            return "node shutting down"

        return self.settle(goal_handle, along, dt)

    def settle(self, goal_handle, along, dt):
        """Command zero velocity until the drone is (almost) at rest."""
        self.get_logger().info(f"Reached the stop point at {along:.1f} m. Settling.")
        deadline = time.monotonic() + SETTLE_TIMEOUT
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return CANCELED
            self.publish()
            pose = self.current_pose()
            if pose is not None and pose.speed < SETTLE_SPEED:
                break
            time.sleep(dt)
        return None


def main(args=None):
    rclpy.init(args=args)
    node = CorridorNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
