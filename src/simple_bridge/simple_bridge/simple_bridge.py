#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import threading
import json
import time
import math
import socket
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None

class SimpleBridge(Node):
    def __init__(self):
        super().__init__('simple_bridge')

        if serial is None:
            self.get_logger().error(
                'Modul pyserial tidak ditemukan. Install dengan: sudo apt install python3-serial'
            )
        
        # ========== SERIAL PORT CONFIG ==========
        # STM32: velocity input (m/s) + command output
        self.declare_parameter('stm32_port', '/dev/ttyUSB1')
        self.stm32_port = self.get_parameter('stm32_port').value
        self.stm32_baud = 115200
        self.stm32_ser = None
        
        # Arduino: IMU yaw input (degrees)
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.arduino_port = self.get_parameter('arduino_port').value
        self.arduino_baud = 115200
        self.arduino_ser = None
        
        # ========== ROS SETUP ==========
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        
        # Subscriber: command dari ROS → STM32
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # ========== STATE ==========
        self.last_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.current_yaw = 0.0  # Dari Arduino IMU (radian)
        self.current_linear_vel = 0.0  # Dari STM32 (m/s)
        
        # ========== TCP SERVER (Optional: Unity) ==========
        self.sensor_port = 8081
        self.cmd_port = 8082
        self.sensor_conn = None
        self.cmd_conn = None
        self.use_unity_tcp = False  # Set True jika masih pakai Unity
        
        # ========== INIT ==========
        self._init_serial_ports()
        self._start_threads()
        
        self.get_logger().info('Simple Bridge siap: STM32 (vel) + Arduino (IMU)')
        self.get_logger().info(f'STM32: {self.stm32_port}, Arduino: {self.arduino_port}')

    def _init_serial_ports(self):
        """Initialize serial connections to STM32 and Arduino."""
        if serial is None:
            return

        # STM32
        try:
            self.stm32_ser = serial.Serial(self.stm32_port, self.stm32_baud, timeout=0.1)
            self.get_logger().info(f'STM32 connected: {self.stm32_port}')
        except Exception as e:
            self.get_logger().warn(f'STM32 tidak terhubung: {e}. Velocity akan 0.')
            self.stm32_ser = None
        
        # Arduino
        try:
            self.arduino_ser = serial.Serial(self.arduino_port, self.arduino_baud, timeout=0.1)
            self.get_logger().info(f'Arduino connected: {self.arduino_port}')
        except Exception as e:
            self.get_logger().warn(f'Arduino tidak terhubung: {e}. IMU akan 0.')
            self.arduino_ser = None

    def _start_threads(self):
        """Start background threads for serial reading."""
        if self.stm32_ser:
            self.stm32_thread = threading.Thread(target=self._read_stm32_loop)
            self.stm32_thread.daemon = True
            self.stm32_thread.start()
        
        if self.arduino_ser:
            self.arduino_thread = threading.Thread(target=self._read_arduino_loop)
            self.arduino_thread.daemon = True
            self.arduino_thread.start()
        
        # Optional: TCP server untuk Unity
        if self.use_unity_tcp:
            self.tcp_sensor_thread = threading.Thread(target=self._tcp_sensor_server)
            self.tcp_sensor_thread.daemon = True
            self.tcp_sensor_thread.start()
            
            self.tcp_cmd_thread = threading.Thread(target=self._tcp_cmd_server)
            self.tcp_cmd_thread.daemon = True
            self.tcp_cmd_thread.start()

    # ========== STM32: READ VELOCITY (m/s) ==========
    def _read_stm32_loop(self):
        """Read velocity (m/s) from STM32 via UART."""
        while rclpy.ok() and self.stm32_ser:
            try:
                if self.stm32_ser.in_waiting > 0:
                    line = self.stm32_ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        vel = self._parse_velocity(line)
                        if vel is not None:
                            self.current_linear_vel = vel
                            self.get_logger().debug(f'[STM32] Velocity: {vel:.3f} m/s')
            except Exception as e:
                self.get_logger().error(f'STM32 read error: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'STM32 general error: {e}')
            time.sleep(0.01)

    def _parse_velocity(self, line):
        """Parse velocity string from STM32."""
        try:
            # Format: "vel:0.25" atau langsung angka "0.25"
            if ':' in line:
                parts = line.split(':')
                if parts[0].lower() == 'vel':
                    return float(parts[1])
            return float(line)
        except ValueError:
            self.get_logger().warn(f'Cannot parse velocity: "{line}"')
            return None

    # ========== ARDUINO: READ IMU YAW (DEGREES → RADIAN) ==========
    def _read_arduino_loop(self):
        """Read IMU yaw (degrees) from Arduino via UART."""
        while rclpy.ok() and self.arduino_ser:
            try:
                if self.arduino_ser.in_waiting > 0:
                    line = self.arduino_ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        yaw_deg = self._parse_yaw(line)
                        if yaw_deg is not None:
                            # ✅ KONVERSI DERAJAT → RADIAN
                            yaw_rad = math.radians(yaw_deg)
                            self.current_yaw = self._normalize_angle(yaw_rad)
                            self._publish_imu(self.current_yaw)
                            self.get_logger().debug(f'[Arduino] Yaw: {yaw_deg:.2f}° = {self.current_yaw:.3f} rad')
            except Exception as e:
                self.get_logger().error(f'Arduino read error: {e}')
                break
            except Exception as e:
                self.get_logger().error(f'Arduino general error: {e}')
            time.sleep(0.01)

    def _parse_yaw(self, line):
        """Parse yaw string from Arduino (input: degrees)."""
        try:
            # Format: "yaw:-160.43" atau langsung angka "-160.43"
            if ':' in line:
                parts = line.split(':')
                if parts[0].lower() == 'yaw':
                    return float(parts[1])
            return float(line)
        except ValueError:
            self.get_logger().warn(f'Cannot parse yaw: "{line}"')
            return None

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _publish_imu(self, yaw):
        """Publish IMU message to /imu topic."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        
        # Orientation from yaw
        msg.orientation.z = math.sin(yaw / 2.0)
        msg.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance: set to 0 for known orientation
        msg.orientation_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 1000.0]
        
        self.imu_pub.publish(msg)

    # ========== ODOMETRY: INTEGRATE VELOCITY + IMU ==========
    def _update_and_publish_odom(self):
        """Integrate velocity + yaw to publish odometry."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if dt > 0 and dt < 1.0:
            # Integrate position using current velocity and yaw
            self.x += self.current_linear_vel * math.cos(self.current_yaw) * dt
            self.y += self.current_linear_vel * math.sin(self.current_yaw) * dt
        
        # Build Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        
        odom.twist.twist.linear.x = self.current_linear_vel
        odom.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.current_yaw / 2.0)
        t.transform.rotation.w = math.cos(self.current_yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)

    # ========== COMMAND: ROS → STM32 ==========
    def cmd_callback(self, msg):
        """Receive /cmd_vel and forward to STM32 via UART."""
        if self.stm32_ser:
            try:
                # ✅ FORMAT TETAP: maju;rotasi_derajat
                maju = 1 if msg.linear.x > 0 else 0
                rotasi_derajat = msg.angular.z * 180.0 / math.pi
                cmd_str = f"{maju};{rotasi_derajat:.1f}\n"
                
                self.stm32_ser.write(cmd_str.encode('utf-8'))
                self.get_logger().debug(f'[CMD→STM32] {cmd_str.strip()}')
            except Exception as e:
                self.get_logger().error(f'Failed to send command to STM32: {e}')

    # ========== TCP SERVER (Optional: Unity) ==========
    def _tcp_sensor_server(self):
        """Optional: Receive sensor data from Unity via TCP (LiDAR, etc)."""
        if not self.use_unity_tcp:
            return
            
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.sensor_port))
        server.listen(1)
        server.settimeout(1.0)
        self.get_logger().info(f'TCP sensor server di port {self.sensor_port}')
        
        while rclpy.ok():
            try:
                conn, addr = server.accept()
                self.get_logger().info(f'Unity sensor connected dari {addr}')
                self.sensor_conn = conn
                self._receive_tcp_sensor_data()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'TCP sensor server error: {e}')
                break

    def _receive_tcp_sensor_data(self):
        """Receive JSON sensor data from Unity (optional: LiDAR)."""
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
                            if 'scan' in msg:
                                self._publish_scan(msg['scan'])
                        except json.JSONDecodeError:
                            self.get_logger().warn('JSON tidak valid dari Unity')
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'TCP sensor receive error: {e}')
                break
        self.get_logger().warn('Koneksi sensor TCP putus')
        self.sensor_conn = None

    def _tcp_cmd_server(self):
        """Optional: Send commands to Unity via TCP."""
        if not self.use_unity_tcp:
            return
            
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.cmd_port))
        server.listen(1)
        server.settimeout(1.0)
        self.get_logger().info(f'TCP command server di port {self.cmd_port}')
        
        while rclpy.ok():
            try:
                conn, addr = server.accept()
                self.get_logger().info(f'Unity command connected dari {addr}')
                self.cmd_conn = conn
                self._send_tcp_commands()
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'TCP command server error: {e}')
                break

    def _send_tcp_commands(self):
        """Optional: Forward commands to Unity."""
        self.cmd_conn.settimeout(0.1)
        while rclpy.ok() and self.cmd_conn:
            try:
                time.sleep(0.1)
            except socket.timeout:
                continue

    def _publish_scan(self, scan_data):
        """Publish LaserScan from Unity (optional)."""
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

    # ========== MAIN LOOP: Publish Odom Periodically ==========
    def spin_loop(self):
        """Main loop: publish odometry at fixed rate."""
        rate = self.create_rate(50)  # 50 Hz odom publish
        while rclpy.ok():
            self._update_and_publish_odom()
            rate.sleep()

    def destroy_node(self):
        """Cleanup serial and TCP connections."""
        if self.stm32_ser:
            self.stm32_ser.close()
        if self.arduino_ser:
            self.arduino_ser.close()
        if self.sensor_conn:
            self.sensor_conn.close()
        if self.cmd_conn:
            self.cmd_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleBridge()
    try:
        node.spin_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()