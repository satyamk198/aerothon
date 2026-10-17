import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus, VehicleGlobalPosition

from takeoff.takeoff_node import COMMAND_QOS, LATCHED_QOS, PX4_QOS, px4_topic

# Tasks run in this order; each is started once the previous one reports success on /task_status.
# Maps task name -> node that executes it (must be subscribed to /task_command before we start).
TASK_SEQUENCE = [
    ("takeoff", "simple_takeoff_node"),
    ("move_forward", "move_forward_node"),
]


class MissionManagement(Node):
    def __init__(self):
        super().__init__("mission_management_node")

        self.task_command_pub = self.create_publisher(String, "/task_command", COMMAND_QOS)
        self.create_subscription(String, "/task_status", self.task_status_callback, LATCHED_QOS)
        self.create_subscription(
            VehicleStatus, px4_topic("/fmu/out/vehicle_status", VehicleStatus),
            self.status_cb, PX4_QOS,
        )
        # Readiness uses streamed topics only: home_position is published once when home is set,
        # and PX4's bridge does not replay it, so a manager started after PX4 would never see it.
        self.create_subscription(
            VehicleGlobalPosition, px4_topic("/fmu/out/vehicle_global_position", VehicleGlobalPosition),
            self.global_pos_cb, PX4_QOS,
        )

        self.connected = False
        self.preflight_ok = False
        self.global_pos_ok = False
        self.task = None  # None -> 'takeoff' -> 'move_forward' -> 'mission_complete' / '<task>_failed'
        self.start_time = self.get_clock().now()
        self.last_waiting = None

        self.readiness_timer = self.create_timer(1.0, self.check_readiness)
        self.get_logger().info("Mission manager started. Checking readiness...")

    def status_cb(self, msg: VehicleStatus):
        self.connected = True
        self.preflight_ok = msg.pre_flight_checks_pass

    def global_pos_cb(self, msg: VehicleGlobalPosition):
        self.global_pos_ok = msg.lat_lon_valid and msg.alt_valid

    def check_readiness(self):
        checks = [
            ("PX4 connected", self.connected),
            ("PX4 global position", self.global_pos_ok),
            ("PX4 pre-flight checks", self.preflight_ok),
        ] + [(f"{node} node", self.node_subscribed(node)) for _, node in TASK_SEQUENCE]
        waiting = [name for name, ok in checks if not ok]

        if waiting:
            if waiting != self.last_waiting:  # log only when the list changes
                self.get_logger().info(f"Waiting for: {', '.join(waiting)}")
                self.last_waiting = waiting
            return

        self.readiness_timer.cancel()
        self.get_logger().info("All checks ready.")
        self.publish_task(TASK_SEQUENCE[0][0])

    def node_subscribed(self, node_name):
        return any(
            info.node_name == node_name
            for info in self.get_subscriptions_info_by_topic("/task_command")
        )

    def publish_task(self, task):
        self.task = task
        self.task_command_pub.publish(String(data=task))
        self.get_logger().info(f"current task: {task} (t+{self.elapsed():.1f}s)")

    def task_status_callback(self, msg: String):
        task, _, result = msg.data.partition(":")
        if task != self.task:
            return  # stale or replayed status from a task that is not running
        if result != "ok":
            self.get_logger().error(f"{task} node reported failure.")
            self.finish(f"{task}_failed")
            return

        self.get_logger().info(f"{task} complete (t+{self.elapsed():.1f}s).")
        tasks = [name for name, _ in TASK_SEQUENCE]
        index = tasks.index(task) + 1
        if index < len(tasks):
            self.publish_task(tasks[index])
        else:
            self.finish("mission_complete")
            self.get_logger().info("Mission complete. Vehicle holding.")

    def finish(self, outcome):
        # Replace the latched task command so restarted task nodes don't re-run anything.
        self.task = outcome
        self.task_command_pub.publish(String(data=outcome))

    def elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
