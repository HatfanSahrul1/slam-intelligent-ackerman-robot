import math
import numpy as np
from .base_state import State
from geometry_msgs.msg import Point, Polygon, Point32, Twist
from visualization_msgs.msg import Marker

class AnalyzeState(State):
    def __init__(self, node):
        super().__init__(node)
        self.node.get_logger().info("AnalyzeState: Menganalisis kontur")
        self.is_done_executing = False

    def on_enter(self):
        # Hentikan robot
        self.stop_robot()
        # Ambil semua data scan yang terkumpul selama explore
        self.scan_data = self.node.scan_buffer  # misal list of (pose, ranges)
        self.process_contour()
        self.compute_center_and_orbit()
        self.publish_visualization()

    def execute(self):
        pass

    def on_exit(self):
        self.node.get_logger().info("AnalyzeState: Selesai")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.node.cmd_pub.publish(twist)

    def process_contour(self):
        # Di sini lakukan clustering atau ekstraksi kontur dari data scan
        # Sederhana: ambil titik-titik halangan dari scan terakhir (atau kumpulan)
        # Misal: konversi scan ke titik-titik dalam frame odom
        self.contour_points = []
        scan = self.node.latest_scan
        if scan:
            angle = scan.angle_min
            for r in scan.ranges:
                if scan.range_min < r < scan.range_max:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    # Transform ke frame odom? Sementara asumsikan di base_link
                    self.contour_points.append(Point(x=x, y=y, z=0.0))
                angle += scan.angle_increment
        self.node.get_logger().info(f"Analyze: Ditemukan {len(self.contour_points)} titik kontur")

    def compute_center_and_orbit(self):
        # Hitung titik tengah dari kontur (misal rata-rata)
        if not self.contour_points:
            self.center = Point(x=0.0, y=0.0, z=0.0)
            self.orbit_radius = 1.0
            return

        sum_x = sum(p.x for p in self.contour_points)
        sum_y = sum(p.y for p in self.contour_points)
        self.center = Point(x=sum_x/len(self.contour_points), y=sum_y/len(self.contour_points), z=0.0)

        # Hitung jarak maksimum dari pusat ke titik kontur (atau rata-rata + margin)
        max_dist = 0.0
        for p in self.contour_points:
            d = math.hypot(p.x - self.center.x, p.y - self.center.y)
            if d > max_dist:
                max_dist = d
        self.orbit_radius = max_dist + 0.5  # tambah margin aman 0.5 m
        self.node.get_logger().info(f"Analyze: Pusat objek di ({self.center.x:.2f}, {self.center.y:.2f}), radius orbit {self.orbit_radius:.2f}")

        # Simpan ke node untuk digunakan state lain
        self.node.object_center = self.center
        self.node.orbit_radius = self.orbit_radius

    def publish_visualization(self):
        # Publikasi marker untuk kontur (polygon) dan pusat (sphere) dan orbit (circle)
        marker_pub = self.node.marker_pub

        # Kontur sebagai titik-titik (atau line strip)
        marker_points = Marker()
        marker_points.header.frame_id = "odom"
        marker_points.header.stamp = self.node.get_clock().now().to_msg()
        marker_points.ns = "contour"
        marker_points.id = 0
        marker_points.type = Marker.POINTS
        marker_points.action = Marker.ADD
        marker_points.pose.orientation.w = 1.0
        marker_points.scale.x = 0.05
        marker_points.scale.y = 0.05
        marker_points.color.a = 1.0
        marker_points.color.g = 1.0
        marker_points.points = self.contour_points
        marker_pub.publish(marker_points)

        # Pusat sebagai sphere
        marker_center = Marker()
        marker_center.header.frame_id = "odom"
        marker_center.header.stamp = self.node.get_clock().now().to_msg()
        marker_center.ns = "center"
        marker_center.id = 1
        marker_center.type = Marker.SPHERE
        marker_center.action = Marker.ADD
        marker_center.pose.position = self.center
        marker_center.pose.orientation.w = 1.0
        marker_center.scale.x = 0.2
        marker_center.scale.y = 0.2
        marker_center.scale.z = 0.2
        marker_center.color.a = 1.0
        marker_center.color.r = 1.0
        marker_pub.publish(marker_center)

        # Orbit sebagai lingkaran (line strip)
        marker_orbit = Marker()
        marker_orbit.header.frame_id = "odom"
        marker_orbit.header.stamp = self.node.get_clock().now().to_msg()
        marker_orbit.ns = "orbit"
        marker_orbit.id = 2
        marker_orbit.type = Marker.LINE_STRIP
        marker_orbit.action = Marker.ADD
        marker_orbit.pose.position = self.center
        marker_orbit.pose.orientation.w = 1.0
        marker_orbit.scale.x = 0.03
        marker_orbit.color.a = 1.0
        marker_orbit.color.b = 1.0

        num_segments = 36
        for i in range(num_segments + 1):
            angle = 2.0 * math.pi * i / num_segments
            x = self.orbit_radius * math.cos(angle)
            y = self.orbit_radius * math.sin(angle)
            p = Point(x=x, y=y, z=0.0)
            marker_orbit.points.append(p)
        marker_pub.publish(marker_orbit)

    def is_done(self):
        return self.is_done_executing

    def next_state(self):
        # Setelah analisis selesai, pindah ke ApproachState
        return "approach"
        # return None