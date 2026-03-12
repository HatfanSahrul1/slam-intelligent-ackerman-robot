import json
import math
from geometry_msgs.msg import Twist
from .base_state import State

class ExploreState(State):
    def __init__(self, node):
        super().__init__(node)
        self.node.get_logger().info("ExploreState: Mengitari objek")
        self.start_pose = None
        self.start_time = None
        self.loop_closed = False
        self.angle_tolerance = 0.1  # radian

        # Load parameters from JSON file
        param_file = "/slam_ws/ws/src/intelligent/intelligent/states/explore_param.json"
        with open(param_file, 'r') as f:
            params = json.load(f)
        self.desired_distance = params.get("desired_distance", 2.5)
        self.tolerance = params.get("tolerance", 1.0)
        self.left_angle = params.get("left", 10)
        self.right_angle = params.get("right", 30)

    def on_enter(self):
        # Jangan set start_pose di sini karena current_pose mungkin masih None
        self.node.get_logger().info(f"Desire distance : {self.desired_distance}\nTolerance : {self.tolerance}\nLeft : {self.left_angle}\nRight : {self.right_angle}")
        self.node.get_logger().info("ExploreState: Mulai mengelilingi objek")

    def on_exit(self):
        self.node.get_logger().info("ExploreState: Selesai")
        twist = Twist()
        self.node.cmd_pub.publish(twist)

    def execute(self):
        if self.node.current_pose is None:
            return
        if self.node.latest_scan_right is None:
            self.node.get_logger().warn("ExploreState: Menunggu scan_right...")
            return

        if self.start_pose is None:
            self.start_pose = self.node.current_pose
            self.start_time = self.node.get_clock().now().nanoseconds
            self.node.get_logger().info(f"Start pose: ({self.start_pose.x:.2f}, {self.start_pose.y:.2f})")

        scan = self.node.latest_scan_right
        ranges = scan.ranges
        valid_ranges = [r for r in ranges if not math.isinf(r) and r > scan.range_min]

        if not valid_ranges:
            # Tidak ada data valid di kanan, putar perlahan untuk mencari objek
            twist = Twist()
            twist.angular.z = 0.5
            self.node.cmd_pub.publish(twist)
            return

        # Ganti min dengan rata-rata
        avg_dist_right = sum(valid_ranges) / len(valid_ranges)
        distance_error = avg_dist_right - self.desired_distance

        self.node.get_logger().info(f"Avg right: {avg_dist_right:.2f}, error: {distance_error:.2f}")

        twist = Twist()
        twist.linear.x = 0.2  # maju konstan

        # Gunakan ambang batas (tolerance) untuk menentukan arah belok
        if distance_error < -self.tolerance:   # terlalu dekat -> belok kiri
            twist.angular.z = -0.3
        elif distance_error > self.tolerance:  # terlalu jauh -> belok kanan
            twist.angular.z = 0.3
        else:
            twist.angular.z = 0.0

        self.node.cmd_pub.publish(twist)


        # # Cek loop closure
        # dx = self.node.current_pose.x - self.start_pose.x
        # dy = self.node.current_pose.y - self.start_pose.y
        # dist_from_start = math.hypot(dx, dy)
        # elapsed = (self.node.get_clock().now().nanoseconds - self.start_time) / 1e9

        # if elapsed > 5.0 and dist_from_start < 0.3:
        #     self.loop_closed = True
        #     self.node.get_logger().info("Loop tertutup!")

    def next_state(self):
        return None
        # return "analyze" if self.loop_closed else None