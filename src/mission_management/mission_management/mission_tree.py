"""AeroTHON 2026 mission as a py_trees behavior tree.

Each step calls an action on flight_controller or on a task node, in order. The whole tree
runs once: if a step fails, the mission stops there and the vehicle keeps holding position.

Steps marked with Placeholder still have to be written; they fail with a log message.

Steps that process camera images are wrapped in WithPhotos: the camera's photo loop
(siyi_a8 photo_node) runs only while such a step runs, not during takeoff, moves or
altitude changes. In SITL there is no photo_node (Gazebo streams the camera all the time);
the wrapper then just runs the step.
"""
import math

import py_trees
import py_trees_ros
import rclpy
from px4_msgs.msg import VehicleGlobalPosition, VehicleStatus
from mission_interfaces.srv import SetPhotoTask
from mission_interfaces.action import (
    AlignWithCorridor, CaptureReference, CenterOnTarget, DeliverPayload, GoTo, Land, NavigateCorridor,
    SearchTarget, Takeoff,
)

from flight_controller.px4_utils import PX4_QOS, px4_topic

# From the mission operation description
QR_SCAN_ALTITUDE = 5.0  # m, scan the start QR
QR_SCAN_ALTITUDE_TOLERANCE = 0.1  # m; takeoff alone only gets within 0.5 m, so climb to it exactly
QR_CAPTURE_TIMEOUT = 60.0  # s to capture a clear image of the start QR
CORRIDOR_ALTITUDE = 3.0  # m, "descend to 10 feet (approximately 3 meter)"
SEARCH_ALTITUDE = 10.0  # m, identify the target QR in the delivery zone
DELIVERY_ALTITUDE = 5.0  # m, payload release
RETURN_ALTITUDE = 10.0  # m, "after payload delivery, ascend to 10 meter altitude"

PHOTO_TASK_SERVICE = "/siyi_photo/set_task"


class WaitForPx4Ready(py_trees.behaviour.Behaviour):
    """RUNNING until PX4 reports a valid global position and passing pre-flight checks."""

    def __init__(self, name="Wait for PX4 ready"):
        super().__init__(name)
        self.preflight_ok = False
        self.global_pos_ok = False

    def setup(self, **kwargs):
        node = kwargs["node"]
        node.create_subscription(
            VehicleStatus, px4_topic("/fmu/out/vehicle_status", VehicleStatus), self.status_cb, PX4_QOS
        )
        node.create_subscription(
            VehicleGlobalPosition, px4_topic("/fmu/out/vehicle_global_position", VehicleGlobalPosition),
            self.global_pos_cb, PX4_QOS,
        )

    def status_cb(self, msg):
        self.preflight_ok = msg.pre_flight_checks_pass

    def global_pos_cb(self, msg):
        self.global_pos_ok = msg.lat_lon_valid and msg.alt_valid

    def update(self):
        if self.preflight_ok and self.global_pos_ok:
            return py_trees.common.Status.SUCCESS
        waiting = [n for n, ok in (("pre-flight checks", self.preflight_ok),
                                   ("global position", self.global_pos_ok)) if not ok]
        self.feedback_message = f"waiting for {', '.join(waiting)}"
        return py_trees.common.Status.RUNNING


class WithPhotos(py_trees.decorators.Decorator):
    """Run the child step with the camera photo loop on, saving into the `task` folder of this
    run; switch it off when the step ends (success, failure or interruption)."""

    def __init__(self, task, child):
        super().__init__(name=f"Photos ({task}): {child.name}", child=child)
        self.task = task
        self.node = None
        self.client = None

    def setup(self, **kwargs):
        self.node = kwargs["node"]
        self.client = self.node.create_client(SetPhotoTask, PHOTO_TASK_SERVICE)

    def initialise(self):
        self.set_task(self.task)

    def update(self):
        return self.decorated.status

    def terminate(self, new_status):
        self.set_task("")

    def set_task(self, task):
        if not self.client.service_is_ready():
            # e.g. SITL, where the Gazebo camera streams continuously
            self.node.get_logger().warn(
                f"{PHOTO_TASK_SERVICE} not available; '{self.decorated.name}' runs without switching photos.")
            return
        self.client.call_async(SetPhotoTask.Request(task=task))


class Placeholder(py_trees.behaviour.Behaviour):
    """A mission step that has not been written yet: fails so the mission stops visibly here."""

    def update(self):
        self.feedback_message = "not implemented"
        self.logger.error(f"'{self.name}' is not implemented yet.")
        return py_trees.common.Status.FAILURE


def action(name, action_type, action_name, goal):
    return py_trees_ros.action_clients.FromConstant(
        name=name, action_type=action_type, action_name=action_name, action_goal=goal
    )


def go_to_altitude(altitude, tolerance=0.0):
    """Straight up/down to `altitude` m above the local origin, keeping horizontal position.
    tolerance <= 0 uses flight_controller's default acceptance radius."""
    return GoTo.Goal(frame=GoTo.Goal.FRAME_LOCAL_NED, x=math.nan, y=math.nan, z=-altitude, yaw=math.nan,
                     acceptance_radius=tolerance)


def create_root():
    mission = py_trees.composites.Sequence("AeroTHON mission", memory=True, children=[
        WaitForPx4Ready(),

        # Takeoff + QR code scan: 5 m, center over the start QR, store a clear image of it
        # (no zoom: the A8 mini does not zoom in 4K mode, and 4K photos are used throughout)
        action(f"Take off to {QR_SCAN_ALTITUDE:.0f} m", Takeoff, "/flight_controller/takeoff",
               Takeoff.Goal(altitude=QR_SCAN_ALTITUDE)),
        action(f"Climb to exactly {QR_SCAN_ALTITUDE:.0f} m", GoTo, "/flight_controller/goto",
               go_to_altitude(QR_SCAN_ALTITUDE, QR_SCAN_ALTITUDE_TOLERANCE)),
        WithPhotos("start_qr", py_trees.composites.Sequence("Start QR", memory=True, children=[
            # As in lawnmower_mission.py: if centring does not converge, hold there and carry on.
            py_trees.decorators.FailureIsSuccess(
                "Carry on if not fully centred",
                action("Center on start QR", CenterOnTarget, "/center_on_target",
                       CenterOnTarget.Goal(target=CenterOnTarget.Goal.TARGET_SINGLE_QR))),
            action("Capture start QR", CaptureReference, "/qr_detector/capture_reference",
                   CaptureReference.Goal(timeout=QR_CAPTURE_TIMEOUT)),
        ])),

        # Corridor navigation: align with the green banner, descend, fly the corridor
        WithPhotos("banner_forward", action("Align with corridor (forward)", AlignWithCorridor,
                                            "/align_with_corridor", AlignWithCorridor.Goal())),
        action(f"Descend to {CORRIDOR_ALTITUDE:.0f} m", GoTo, "/flight_controller/goto",
               go_to_altitude(CORRIDOR_ALTITUDE)),
        WithPhotos("corridor_forward", action("Navigate corridor (forward)", NavigateCorridor,
                                              "/navigate_corridor", NavigateCorridor.Goal())),

        # Delivery zone navigation: climb and find the QR matching the captured one
        action(f"Climb to {SEARCH_ALTITUDE:.0f} m", GoTo, "/flight_controller/goto", go_to_altitude(SEARCH_ALTITUDE)),
        WithPhotos("target_search", action("Search target QR", SearchTarget, "/search_target", SearchTarget.Goal())),

        # Payload delivery
        action(f"Descend to {DELIVERY_ALTITUDE:.0f} m", GoTo, "/flight_controller/goto",
               go_to_altitude(DELIVERY_ALTITUDE)),
        WithPhotos("center_deliver", py_trees.composites.Sequence("Center and deliver", memory=True, children=[
            action("Center on target", CenterOnTarget, "/center_on_target",
                   CenterOnTarget.Goal(target=CenterOnTarget.Goal.TARGET_MATCHED_QR)),
            action("Deliver payload", DeliverPayload, "/deliver_payload", DeliverPayload.Goal()),
        ])),

        # Return flight
        action(f"Climb to {RETURN_ALTITUDE:.0f} m", GoTo, "/flight_controller/goto", go_to_altitude(RETURN_ALTITUDE)),
        # TODO: return lap details still to be specified
        WithPhotos("corridor_return", Placeholder("Return through corridor")),
        Placeholder("Return to take-off point"),  # TODO: record the take-off position, GoTo it

        # Landing
        action("Land", Land, "/flight_controller/land", Land.Goal()),
    ])
    # Run the sequence once; without this the tree would restart the mission after it finishes.
    return py_trees.decorators.OneShot(
        name="Run once", child=mission, policy=py_trees.common.OneShotPolicy.ON_COMPLETION
    )


def main(args=None):
    rclpy.init(args=args)
    tree = py_trees_ros.trees.BehaviourTree(create_root())
    reported = []

    def report(tree):
        status = tree.root.status
        if status in (py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE) and not reported:
            reported.append(status)
            tip = tree.root.tip()
            if status == py_trees.common.Status.SUCCESS:
                tree.node.get_logger().info("MISSION COMPLETE.")
            else:
                tree.node.get_logger().error(f"MISSION FAILED at '{tip.name if tip else '?'}'.")

    try:
        # Blocks until every action server in the tree is available.
        tree.setup(node_name="mission_tree")
        tree.tick_tock(period_ms=100, post_tick_handler=report)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        tree.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
