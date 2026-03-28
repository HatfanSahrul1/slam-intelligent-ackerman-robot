import math
from .base_state import State
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from intelligent import utils

class ApproachState(State):
    def __init__(self, node):
        super().__init__(node)
        self.node.get_logger().info("ApproachState: Menuju orbit")
        self.is_done_executing = False

    def on_enter(self):
        self.target_reached = False
        self.center = self.node.object_center
        self.orbit_radius = self.node.orbit_radius
        # Hitung titik di orbit yang paling dekat dengan robot saat ini
        self.target_point = self.find_entry_point()
        self.node.get_logger().info(f"Target entry point: ({self.target_point.x:.2f}, {self.target_point.y:.2f})")

    def on_exit(self):
        self.node.get_logger().info("ApproachState: Selesai")

    def find_entry_point(self):
        # Cari titik pada lingkaran orbit yang paling dekat dengan posisi robot
        robot = self.node.current_pose
        dx = robot.x - self.center.x
        dy = robot.y - self.center.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            angle = 0.0
        else:
            angle = math.atan2(dy, dx)
        # Titik pada lingkaran dengan arah yang sama dari pusat
        px = self.center.x + self.orbit_radius * math.cos(angle)
        py = self.center.y + self.orbit_radius * math.sin(angle)
        return Point(x=px, y=py, z=0.0)

    def execute(self):
        robot = self.node.current_pose
        dx = self.target_point.x - robot.x
        dy = self.target_point.y - robot.y
        distance = math.hypot(dx, dy)

        if distance < 0.1:
            self.target_reached = True
            # Hentikan robot
            twist = Twist()
            self.node.cmd_pub.publish(twist)
            return

        # Hitung sudut target relatif terhadap robot
        target_angle = math.atan2(dy, dx)
        # Kurangi dengan yaw robot
        angle_diff = utils.normalize_angle(target_angle - self.node.current_yaw)

        # Kontrol sederhana P
        angular_gain = 0.8
        linear_gain = 0.3
        twist = Twist()
        twist.linear.x = min(linear_gain * distance, 0.3)  # batasi kecepatan
        twist.angular.z = angular_gain * angle_diff

        self.node.cmd_pub.publish(twist)

    def is_done(self):
        return self.is_done_executing

    def next_state(self):
        if self.target_reached:
            return "orbit"
        return None