#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import socket
import threading
import json
import time
import math

# ✅ IMPORT MODUL EKSTERNAL
from .odometry_calculator import OdometryCalculator
from tf2_ros import TransformBroadcaster

class SimpleBridge(Node):
    def __init__(self):
        super().__init__('simple_bridge')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sensor_port = 8081
        self.cmd_port = 8082
        self.sensor_conn = None
        self.cmd_conn = None
        
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # ✅ ODOMETRY CALCULATOR (untuk mode kalkulasi, optional)
        self.odom_calculator = OdometryCalculator(
            config_path='/slam_ws/ws/src/simple_bridge/config/bridge_params.json',
            node=self
        )
        
        # ✅ FLAG: Gunakan ground truth odom langsung dari Unity
        self.use_ground_truth_odom = True
        
        self.sensor_thread = threading.Thread(target=self.sensor_server)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()
        
        self.cmd_thread = threading.Thread(target=self.cmd_server)
        self.cmd_thread.daemon = True
        self.cmd_thread.start()
        
        self.get_logger().info('Simple Bridge siap. Menunggu Unity...')
        self.get_logger().info(f'Ground truth odom mode: {self.use_ground_truth_odom}')

    def sensor_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.sensor_port))
        server.listen(1)
        server.settimeout(1.0)
        self.get_logger().info(f'Sensor server di port {self.sensor_port}')
        
        while rclpy.ok():
            try:
                conn, addr = server.accept()
                self.get_logger().info(f'Unity sensor connected dari {addr}')
                self.sensor_conn = conn
                self.receive_sensor_data()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Sensor server error: {e}')
                break

    def receive_sensor_data(self):
        buffer = ''
        self.sensor_conn.settimeout(0.1)
        while rclpy.ok() and self.sensor_conn:
            try:
                data = self.sensor_conn.recv(4096).decode('utf-8')
                if not data:
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            msg = json.loads(line)
                            
                            # ✅ MODE GROUND TRUTH: Publish odom langsung dari Unity
                            if self.use_ground_truth_odom and 'odom' in msg:
                                self.publish_ground_truth_odom(msg['odom'])
                            
                            # ✅ MODE KALKULASI: Hitung odom dari twist (fallback/opsional)
                            elif 'twist' in msg and 'imu' in msg:
                                current_time = self.get_clock().now()
                                result = self.odom_calculator.update(
                                    msg['twist'],
                                    msg['imu']['theta'],
                                    current_time
                                )
                                self.odom_pub.publish(result['odom_msg'])
                                self.tf_broadcaster.sendTransform(result['tf_msg'])
                            
                            if 'scan' in msg:
                                self.publish_scan(msg['scan'])
                        except json.JSONDecodeError:
                            self.get_logger().warn('JSON tidak valid')
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Error receive sensor: {e}')
                break
        self.get_logger().warn('Koneksi sensor putus')
        self.sensor_conn = None

    def publish_ground_truth_odom(self, odom_data):
        """
        Publish odometry langsung dari ground truth Unity.
        Tanpa integrasi, tanpa kalkulasi tambahan.
        """
        current_time = self.get_clock().now()
        
        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # ✅ LANGSUNG PAKAI NILAI DARI UNITY
        odom.pose.pose.position.x = float(odom_data.get('x', 0.0))
        odom.pose.pose.position.y = float(odom_data.get('y', 0.0))
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(odom_data.get('theta', 0.0) / 2.0)
        odom.pose.pose.orientation.w = math.cos(odom_data.get('theta', 0.0) / 2.0)
        
        # Twist bisa tetap dari robot (untuk kontrol) atau 0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)
        
        # Publish TF transform (juga langsung dari ground truth)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

    def publish_scan(self, scan_data):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = scan_data['angle_min']
        scan.angle_max = scan_data['angle_max']
        scan.angle_increment = scan_data['angle_increment']
        scan.range_min = scan_data['range_min']
        scan.range_max = scan_data['range_max']
        scan.ranges = scan_data['ranges']
        self.scan_pub.publish(scan)

    def cmd_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.cmd_port))
        server.listen(1)
        server.settimeout(1.0)
        self.get_logger().info(f'Command server di port {self.cmd_port}')
        
        while rclpy.ok():
            try:
                conn, addr = server.accept()
                self.get_logger().info(f'Unity command connected dari {addr}')
                self.cmd_conn = conn
                self.send_commands()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Command server error: {e}')
                break

    def send_commands(self):
        self.cmd_conn.settimeout(0.1)
        while rclpy.ok() and self.cmd_conn:
            try:
                time.sleep(0.1)
            except socket.timeout:
                continue

    def cmd_callback(self, msg):
        if self.cmd_conn:
            try:
                maju = 1 if msg.linear.x > 0 else 0
                rotasi_derajat = msg.angular.z * 180.0 / math.pi
                cmd_str = f"{maju};{rotasi_derajat:.1f}\n"
                self.cmd_conn.sendall(cmd_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f'Gagal kirim command: {e}')

    # ============================================================
    # ✅ KODE LAMA DI-COMMENT (TIDAK DIHAPUS)
    # ============================================================
    """
    def update_odometry(self, twist, imu_theta):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt > 0:
            vx_raw = twist.get('linear_x', 0.0)
            wz_raw = twist.get('angular_z', 0.0)
            
            vx = vx_raw * self.odom_scale_linear
            wz = wz_raw * self.odom_scale_angular
            
            self.x += vx * math.cos(imu_theta) * dt
            self.y += vx * math.sin(imu_theta) * dt
            self.last_time = current_time

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(imu_theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(imu_theta / 2.0)
        odom.twist.twist.linear.x = twist.get('linear_x', 0.0) * self.odom_scale_linear
        odom.twist.twist.angular.z = twist.get('angular_z', 0.0) * self.odom_scale_angular

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(imu_theta/2.0)
        t.transform.rotation.w = math.cos(imu_theta/2.0)
        self.tf_broadcaster.sendTransform(t)

        self.odom_pub.publish(odom)
    """

    def destroy_node(self):
        if self.sensor_conn:
            self.sensor_conn.close()
        if self.cmd_conn:
            self.cmd_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()