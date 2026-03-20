#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import json
import os

class TuningNode(Node):
    def __init__(self):
        super().__init__('tuning_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.state = 'idle'
        self.start_time = None
        self.forward_duration = 5.0
        self.turn_duration = 5.0
        self.linear_speed_cmd = 1.0   # trigger untuk maju (nilai >0)
        self.angular_speed_cmd = 0.5  # trigger untuk belok
        self.odom_positions = []
        self.odom_yaws = []
        self.current_odom = None
        self.get_logger().info('Tuning node siap. Pastikan robot di Unity sudah siap.')
        self.tuning_timer = self.create_timer(2.0, self.start_tuning_callback)

    def odom_callback(self, msg):
        self.current_odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z)
        cosy_cosp = 1 - 2 * (q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_positions.append((x, y))
        self.odom_yaws.append(yaw)
        self.get_logger().info(f"Odom: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

    def control_loop(self):
        if self.state == 'idle':
            pass
        elif self.state == 'forward_start':
            self.start_time = time.time()
            self.odom_positions.clear()
            self.odom_yaws.clear()
            self.state = 'forward'
            self.get_logger().info("Mulai maju lurus selama {} detik".format(self.forward_duration))
        elif self.state == 'forward':
            elapsed = time.time() - self.start_time
            if elapsed < self.forward_duration:
                twist = Twist()
                twist.linear.x = self.linear_speed_cmd
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.state = 'forward_done'
                self.get_logger().info("Selesai maju. Catat posisi ground truth dari Unity.")
                self.prompt_for_factor('linear')
        elif self.state == 'turn_start':
            self.start_time = time.time()
            self.odom_positions.clear()
            self.odom_yaws.clear()
            self.state = 'turn'
            self.get_logger().info("Mulai belok sambil maju selama {} detik".format(self.turn_duration))
        elif self.state == 'turn':
            elapsed = time.time() - self.start_time
            if elapsed < self.turn_duration:
                twist = Twist()
                twist.linear.x = self.linear_speed_cmd
                twist.angular.z = self.angular_speed_cmd
                self.cmd_pub.publish(twist)
            else:
                self.cmd_pub.publish(Twist())
                self.state = 'turn_done'
                self.get_logger().info("Selesai belok. Catat perubahan yaw ground truth dari Unity.")
                self.prompt_for_factor('angular')
        elif self.state == 'done':
            self.get_logger().info("Tuning selesai. Faktor telah disimpan.")

    def prompt_for_factor(self, mode):
        if not self.odom_positions:
            self.get_logger().error("Tidak ada data odom.")
            return
        if mode == 'linear':
            x0, y0 = self.odom_positions[0]
            x1, y1 = self.odom_positions[-1]
            dist_odom = math.hypot(x1 - x0, y1 - y0)
            self.get_logger().info(f"Jarak tempuh menurut odom ROS: {dist_odom:.3f} m")
            try:
                gt_dist = float(input("Masukkan jarak tempuh dari Unity (ground truth) dalam meter: "))
                if gt_dist > 0:
                    factor = gt_dist / dist_odom
                    self.get_logger().info(f"Faktor skala linear = {factor:.3f}")
                    self.save_factor('linear', factor)
            except:
                self.get_logger().error("Input tidak valid.")
        elif mode == 'angular':
            if not self.odom_yaws:
                return
            yaw0 = self.odom_yaws[0]
            yaw1 = self.odom_yaws[-1]
            delta = yaw1 - yaw0
            delta = math.atan2(math.sin(delta), math.cos(delta))
            delta_deg = delta * 180 / math.pi
            self.get_logger().info(f"Perubahan yaw menurut odom ROS: {delta:.3f} rad ({delta_deg:.1f} deg)")
            try:
                gt_delta = float(input("Masukkan perubahan yaw dari Unity (ground truth) dalam radian: "))
                if gt_delta != 0:
                    factor = gt_delta / delta
                    self.get_logger().info(f"Faktor skala angular = {factor:.3f}")
                    self.save_factor('angular', factor)
            except:
                self.get_logger().error("Input tidak valid.")
        if mode == 'linear':
            self.state = 'turn_start'
        elif mode == 'angular':
            self.state = 'done'

    def save_factor(self, mode, factor):
        config_path = os.path.expanduser('/slam_ws/ws/src/simple_bridge/config/bridge_params.json')
        try:
            with open(config_path, 'r') as f:
                params = json.load(f)
        except:
            params = {}
        if mode == 'linear':
            params['odom_scale_linear'] = factor
        else:
            params['odom_scale_angular'] = factor
        with open(config_path, 'w') as f:
            json.dump(params, f, indent=4)
        self.get_logger().info(f"Faktor {mode} disimpan ke {config_path}")

    def start_tuning(self):
        self.state = 'forward_start'
        self.get_logger().info("Memulai tuning...")

    def start_tuning_callback(self):
        self.start_tuning()
        self.destroy_timer(self.tuning_timer)

def main(args=None):
    rclpy.init(args=args)
    node = TuningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()