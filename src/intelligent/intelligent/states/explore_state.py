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

    def on_enter(self):
        # Jangan set start_pose di sini karena current_pose mungkin masih None
        self.node.get_logger().info("ExploreState: Mulai mengelilingi objek")

    def on_exit(self):
        self.node.get_logger().info("ExploreState: Selesai")
        twist = Twist()
        self.node.cmd_pub.publish(twist)

    def execute(self):
        # Pastikan data tersedia
        if self.node.current_pose is None:
            return
        if self.node.latest_scan_right is None:
            self.node.get_logger().warn("ExploreState: Menunggu scan_right...")
            return

        # Inisialisasi start_pose jika belum (setelah data pose tersedia)
        if self.start_pose is None:
            self.start_pose = self.node.current_pose
            self.start_time = self.node.get_clock().now().nanoseconds
            self.node.get_logger().info(f"Start pose: ({self.start_pose.x:.2f}, {self.start_pose.y:.2f})")

        # Ambil data dari scan_right
        scan = self.node.latest_scan_right
        ranges = scan.ranges
        angle_min = scan.angle_min
        angle_max = scan.angle_max
        angle_increment = scan.angle_increment

        # Kita asumsikan scan_right sudah berisi data di sisi kanan.
        # Namun kita perlu menentukan indeks yang mewakili arah tegak lurus kanan (misal 90° dari forward).
        # Karena scan_right mungkin mencakup rentang sudut yang lebar, kita bisa mengambil nilai tengahnya.
        # Alternatif: cari nilai minimum di seluruh scan_right untuk deteksi halangan terdekat.
        # Di sini kita gunakan nilai minimum sebagai indikator jarak ke objek terdekat di kanan.
        valid_ranges = [r for r in ranges if not math.isinf(r) and r > scan.range_min]
        if valid_ranges:
            min_dist_right = min(valid_ranges)
        else:
            min_dist_right = float('inf')

        self.node.get_logger().info(f"Min distance right: {min_dist_right:.2f}")
        
        desired_distance = 2.5  # meter
        tolerance = 1         # meter (zona mati untuk lurus)

        # Hitung selisih jarak
        distance_error = min_dist_right - desired_distance

        # Tentukan target_angle berdasarkan zona
        if distance_error < -tolerance:  
            # Terlalu dekat (< 1.3m) -> Belok Kiri Menjauh
            target_angle = self.node.current_yaw + math.radians(10)
        elif distance_error > tolerance: 
            # Terlalu jauh (> 1.7m) -> Belok Kanan Mendekat
            target_angle = self.node.current_yaw - math.radians(30)
        else:
            # Zona Ideal (1.3m - 1.7m) -> MAJU LURUS
            target_angle = self.node.current_yaw 

        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))
        angle_error = target_angle - self.node.current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()
        twist.linear.x = 0.2  # kecepatan maju konstan
        if abs(angle_error) > self.angle_tolerance:
            twist.angular.z = 0.5 * angle_error
        else:
            twist.angular.z = 0.0

        self.node.cmd_pub.publish(twist)

        # Cek loop closure
        dx = self.node.current_pose.x - self.start_pose.x
        dy = self.node.current_pose.y - self.start_pose.y
        dist_from_start = math.hypot(dx, dy)
        elapsed = (self.node.get_clock().now().nanoseconds - self.start_time) / 1e9

        # Loop dianggap tertutup jika kembali ke posisi awal setelah minimal 5 detik dan jarak < 0.3 m
        if elapsed > 5.0 and dist_from_start < 0.3:
            self.loop_closed = True
            self.node.get_logger().info("Loop tertutup!")

    def next_state(self):
        return None
        # return "analyze" if self.loop_closed else None