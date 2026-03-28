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

# ✅ IMPORT UNIFIED CLASS
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
        
        # ✅ INITIALIZE CALCULATOR (config loaded automatically)
        self.odom_calculator = OdometryCalculator(
            config_path='/slam_ws/ws/src/simple_bridge/config/bridge_params.json',
            node=self
        )
        
        self.sensor_thread = threading.Thread(target=self.sensor_server)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()
        
        self.cmd_thread = threading.Thread(target=self.cmd_server)
        self.cmd_thread.daemon = True
        self.cmd_thread.start()
        
        self.get_logger().info('Simple Bridge siap. Menunggu Unity...')

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
                            if 'twist' in msg and 'imu' in msg:
                                # ✅ CALCULATE ODOMETRY
                                current_time = self.get_clock().now()
                                result = self.odom_calculator.update(
                                    msg['twist'],
                                    msg['imu']['theta'],
                                    current_time
                                )
                                
                                # ✅ PUBLISH ODOMETRY (simple_bridge handles publishing)
                                self.odom_pub.publish(result['odom_msg'])
                                self.tf_broadcaster.sendTransform(result['tf_msg'])
                                
                                # Optional: log position
                                # self.get_logger().debug(f"Pos: x={result['x']:.2f}, y={result['y']:.2f}")
                            
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