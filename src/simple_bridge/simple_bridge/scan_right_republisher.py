#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanRightRepublisher(Node):
    def __init__(self):
        super().__init__('scan_right_republisher')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_right', 10)

    def scan_callback(self, msg):
        # Salin header dan parameter
        new_scan = LaserScan()
        new_scan.header = msg.header
        new_scan.angle_min = msg.angle_min
        new_scan.angle_max = msg.angle_max
        new_scan.angle_increment = msg.angle_increment
        new_scan.range_min = msg.range_min
        new_scan.range_max = msg.range_max
        new_scan.time_increment = msg.time_increment
        new_scan.scan_time = msg.scan_time
        new_scan.intensities = msg.intensities

        # Rentang sudut untuk sisi kanan (225° sampai 315°, atau -135° sampai -45° dalam bentuk positif)
        # Karena angle_min=0, kita gunakan sudut positif
        right_min_angle = 225.0 * math.pi / 180.0   # 3.92699 rad
        right_max_angle = 315.0 * math.pi / 180.0   # 5.49779 rad

        # Hitung indeks
        idx_min = int((right_min_angle - msg.angle_min) / msg.angle_increment)
        idx_max = int((right_max_angle - msg.angle_min) / msg.angle_increment)

        # Pastikan indeks dalam batas
        idx_min = max(0, min(idx_min, len(msg.ranges)-1))
        idx_max = max(0, min(idx_max, len(msg.ranges)-1))

        # Buat array ranges baru dengan nilai inf di luar rentang
        new_ranges = [float('inf')] * len(msg.ranges)
        for i in range(idx_min, idx_max+1):
            new_ranges[i] = msg.ranges[i]

        new_scan.ranges = new_ranges
        self.pub.publish(new_scan)

def main(args=None):
    rclpy.init(args=args)
    node = ScanRightRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()