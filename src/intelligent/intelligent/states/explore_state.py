import json
import math
from geometry_msgs.msg import Twist
from .base_state import State
from ..motion import MoveToOdom

class ExploreState(State):
    def __init__(self, node):
        super().__init__(node)
        self.node.get_logger().info("ExploreState: Initialized")
        
        # Load parameters
        param_file = "/slam_ws/ws/src/intelligent/intelligent/states/explore_param.json"
        with open(param_file, 'r') as f:
            params = json.load(f)
        
        self.mode = params.get("mode", "lidar")
        self.node.get_logger().info(f"MODE AKTIF: {self.mode.upper()}")

        # --- Parameter Odom Mode ---
        self.waypoints = params.get("waypoints", [])
        self.current_wp_index = 0
        self.wp_threshold = params.get("waypoint_threshold", 0.3)
        self.nav_speed = params.get("linear_speed", 0.5)

        # --- Parameter Lidar Mode ---
        self.desired_distance = params.get("desired_distance", 3.5)
        self.tolerance = params.get("tolerance", 1.0)
        self.left_angle = params.get("left", 10)
        self.right_angle = params.get("right", 12)

        # State variables
        self.start_pose = None
        self.angle_tolerance = 0.1

    def on_enter(self):
        self.node.get_logger().info(f"ExploreState: Entering {self.mode.upper()} mode")
        if self.mode == "odom":
            self.current_wp_index = 0
            self.node.get_logger().info(f"Total waypoints: {len(self.waypoints)}")

    def on_exit(self):
        self.node.get_logger().info("ExploreState: Exit")
        twist = Twist()
        self.node.cmd_pub.publish(twist)

    def execute(self):
        if self.node.current_pose is None:
            return

        if self.mode == "odom":
            self.execute_odom_mode()
        else:
            self.execute_lidar_mode()

    # ============================================================
    # MODE 1: ODOMETRY NAVIGATION (Waypoint Follower)
    # ============================================================
    def execute_odom_mode(self):
        if not self.waypoints:
            return

        if self.current_wp_index >= len(self.waypoints):
            self.current_wp_index = 0

        target_x, target_y = self.waypoints[self.current_wp_index]
        
        # Panggil fungsi MoveToOdom
        motion, angular_cmd, is_done = MoveToOdom(
            x_target=target_x,
            y_target=target_y,
            x_input=self.node.current_pose.x,
            y_input=self.node.current_pose.y,
            theta_input=self.node.current_yaw,
            threshold=self.wp_threshold
        )

        self.node.get_logger().info(f"Pos Current [{self.node.current_pose.x}, {self.node.current_pose.y}, {self.node.current_yaw}]")

        if is_done:
            self.node.get_logger().info(f"Waypoint {self.current_wp_index} REACHED!")
            self.current_wp_index += 1
            return

        # Kirim command ke robot
        twist = Twist()
        twist.linear.x = float(self.nav_speed) if motion == 1 else 0.0
        twist.angular.z = float(angular_cmd) if angular_cmd is not None else 0.0

        self.node.cmd_pub.publish(twist)

    # ============================================================
    # MODE 2: LIDAR FOLLOWING (Wall/Object Follower)
    # ============================================================
    def execute_lidar_mode(self):
        if self.node.latest_scan_right is None:
            self.node.get_logger().warn("Lidar Mode: Waiting for scan_right...")
            return

        if self.start_pose is None:
            self.start_pose = self.node.current_pose

        scan = self.node.latest_scan_right
        ranges = scan.ranges
        valid_ranges = [r for r in ranges if not math.isinf(r) and r > scan.range_min]

        if not valid_ranges:
            twist = Twist()
            twist.angular.z = 0.5  # Putar cari objek
            self.node.cmd_pub.publish(twist)
            return

        avg_dist_right = sum(valid_ranges) / len(valid_ranges)
        distance_error = avg_dist_right - self.desired_distance

        if distance_error < -self.tolerance:   # Terlalu dekat -> belok kiri
            target_angle = self.node.current_yaw + math.radians(self.left_angle)
        elif distance_error > self.tolerance:  # Terlalu jauh -> belok kanan
            target_angle = self.node.current_yaw - math.radians(self.right_angle)
        else:
            target_angle = self.node.current_yaw

        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
        angle_error = target_angle - self.node.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()
        twist.linear.x = 0.2
        angular_gain = 0.5
        
        if abs(angle_error) > self.angle_tolerance:
            twist.angular.z = angular_gain * angle_error
        else:
            twist.angular.z = 0.0

        self.node.cmd_pub.publish(twist)

    def next_state(self):
        return None