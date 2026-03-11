import math
from .base_state import State
from geometry_msgs.msg import Twist
from intelligent import utils

class OrbitState(State):
    def __init__(self, node):
        super().__init__(node)
        self.node.get_logger().info("OrbitState: Mengorbit objek")

    def on_enter(self):
        self.center = self.node.object_center
        self.radius = self.node.orbit_radius
        # Tentukan arah orbit (misal clockwise)
        self.angular_speed = -0.2  # rad/s (negatif = clockwise)
        self.linear_speed = abs(self.angular_speed) * self.radius  # v = ω * r
        self.start_angle = self.get_angle_from_robot()
        self.angle_travelled = 0.0
        self.prev_angle = self.start_angle
        self.node.get_logger().info(f"Mulai orbit dengan linear {self.linear_speed:.2f} m/s")

    def on_exit(self):
        self.node.get_logger().info("OrbitState: Selesai")
        # Hentikan robot
        twist = Twist()
        self.node.cmd_pub.publish(twist)

    def get_angle_from_robot(self):
        robot = self.node.current_pose
        dx = robot.x - self.center.x
        dy = robot.y - self.center.y
        return math.atan2(dy, dx)

    def execute(self):
        # Hitung sudut saat ini relatif terhadap pusat
        current_angle = self.get_angle_from_robot()
        # Selisih sudut dengan sebelumnya, perhatikan wrap-around
        delta_angle = utils.normalize_angle(current_angle - self.prev_angle)
        self.angle_travelled += abs(delta_angle)  # akumulasi total sudut (nilai absolut)
        self.prev_angle = current_angle

        # Kontrol kecepatan untuk menjaga orbit
        # Idealnya robot harus bergerak dengan kecepatan linear dan angular konstan
        # Namun karena mungkin ada gangguan, kita bisa menggunakan kontrol sederhana
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.node.cmd_pub.publish(twist)

        # Cek apakah sudah menempuh 360° (dengan toleransi)
        if self.angle_travelled >= 2.0 * math.pi - 0.1:
            self.node.get_logger().info("Orbit 360° selesai")
            # Berhenti dan mungkin kembali ke state awal atau selesai
            self.stop_robot()
            self.node.shutdown_flag = True  # misal untuk mengakhiri node

    def stop_robot(self):
        twist = Twist()
        self.node.cmd_pub.publish(twist)

    def next_state(self):
        # Setelah selesai orbit, bisa kembali ke explore atau berhenti
        # Untuk sekarang, tidak ada state berikutnya
        return None