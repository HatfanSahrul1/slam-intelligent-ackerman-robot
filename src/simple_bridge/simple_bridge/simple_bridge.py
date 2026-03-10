#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import threading
import json
import time
import math

class SimpleBridge(Node):
    def __init__(self):
        super().__init__('simple_bridge')
        self.sensor_port = 8081
        self.cmd_port = 8082
        self.sensor_conn = None
        self.cmd_conn = None
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
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
                data = self.sensor_conn.recv(1024).decode('utf-8')
                if not data:
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            msg = json.loads(line)
                            self.get_logger().info(f'Dari Unity: {msg}')
                        except:
                            self.get_logger().warn('JSON tidak valid')
            except socket.timeout:
                continue
            except:
                break
        self.get_logger().warn('Koneksi sensor putus')
        self.sensor_conn = None

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
                # Format: maju(1/0);target_rotasi(derajat)
                maju = 1 if msg.linear.x > 0 else 0
                # Konversi angular.z dari radian ke derajat (asumsi sebagai kecepatan sudut)
                # Jika ingin target rotasi absolut, perlu diubah
                rotasi_derajat = msg.angular.z * 180.0 / math.pi
                cmd_str = f"{maju};{rotasi_derajat:.1f}\n"
                self.cmd_conn.sendall(cmd_str.encode('utf-8'))
                self.get_logger().info(f'Kirim ke Unity: {cmd_str.strip()}')
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