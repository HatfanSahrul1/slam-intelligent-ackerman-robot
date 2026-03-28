#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import tf_transformations
from .states.explore_state import ExploreState
from .states.analyze_state import AnalyzeState
from .states.approach_state import ApproachState
from .states.orbit_state import OrbitState

class IntelligentNode(Node):
    def __init__(self):
        super().__init__('intelligent_node')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/intelligent/markers', 10)

        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(LaserScan, '/scan_right', self.scan_right_callback, 10)

        # Data terbaru
        self.current_pose = None
        self.current_yaw = 0.0
        self.latest_scan = None
        self.latest_scan_right = None
        self.scan_buffer = []  # untuk menyimpan data scan selama explore

        # Hasil analisis
        self.object_center = Point(x=0.0, y=0.0, z=0.0)
        self.orbit_radius = 1.0

        # State machine
        self.states = {
            'explore': ExploreState(self),
            'analyze': AnalyzeState(self),
            'approach': ApproachState(self),
            'orbit': OrbitState(self)
        }
        self.current_state = 'explore'
        self.states[self.current_state].on_enter()
        self.create_timer(0.1, self.update)  # 10 Hz

        self.get_logger().info('Intelligent node started. Initial state: explore')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def scan_callback(self, msg):
        self.latest_scan = msg
        # Simpan scan untuk analisis nanti (jika dalam explore state)
        if self.current_state == 'explore':
            self.scan_buffer.append((self.current_pose, msg))  # simpan pose juga

    def scan_right_callback(self, msg):
        self.latest_scan_right = msg

    def update(self):
        if not self.current_pose or not self.latest_scan:
            return

        # Eksekusi state saat ini
        self.states[self.current_state].execute()

        # Cek transisi
        if self.states[self.current_state].is_done():
            next_state = self.states[self.current_state].next_state()
            if next_state and next_state in self.states:
                self.get_logger().info(f'Transisi dari {self.current_state} ke {next_state}')
                self.states[self.current_state].on_exit()
                self.current_state = next_state
                self.states[self.current_state].on_enter()

def main(args=None):
    rclpy.init(args=args)
    node = IntelligentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()