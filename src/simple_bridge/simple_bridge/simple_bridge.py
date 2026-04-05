#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import threading
import time
import math
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
                'Modul pyserial tidak ditemukan. Install dengan: pip3 install pyserial'
            )
        
        # ========== SERIAL PORT CONFIG ==========
        # STM32: velocity input (m/s) + command output
        self.declare_parameter('stm32_port', '/dev/ttyUSB0')  # ✅ Sesuai test script Anda
        self.stm32_port = self.get_parameter('stm32_port').value
        self.stm32_baud = 115200
        self.stm32_ser = None
        
        # Arduino: IMU yaw input (degrees)
        self.declare_parameter('arduino_port', '/dev/ttyUSB1')
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
            self.stm32_ser = serial.Serial(
                port=self.stm32_port,
                baudrate=self.stm32_baud,
                timeout=0.1  # ✅ Sama seperti test script
            )
            self.get_logger().info(f'STM32 connected: {self.stm32_port}')
        except Exception as e:
            self.get_logger().warn(f'STM32 tidak terhubung: {e}. Velocity akan 0.')
            self.stm32_ser = None
        
        # Arduino
        try:
            self.arduino_ser = serial.Serial(
                port=self.arduino_port,
                baudrate=self.arduino_baud,
                timeout=0.1
            )
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
            self.get_logger().info(f'STM32 read thread started: {self.stm32_port}')
        
        if self.arduino_ser:
            self.arduino_thread = threading.Thread(target=self._read_arduino_loop)
            self.arduino_thread.daemon = True
            self.arduino_thread.start()
            self.get_logger().info(f'Arduino read thread started: {self.arduino_port}')

    # ========== STM32: READ VELOCITY (m/s) ==========
    def _read_stm32_loop(self):
        """Read velocity (m/s) from STM32 via UART - sama seperti test script."""
        while rclpy.ok() and self.stm32_ser:
            try:
                # ✅ SAMA PERSIS DENGAN TEST SCRIPT ANDA
                raw = self.stm32_ser.readline()
                if raw:
                    line = raw.decode('utf-8', errors='replace').rstrip('\r\n')
                    if line:
                        vel = self._parse_velocity(line)
                        if vel is not None:
                            self.current_linear_vel = vel
                            self.get_logger().debug(f'[STM32] Velocity: {vel:.3f} m/s')
                else:
                    time.sleep(0.01)  # Hindari busy loop
            except Exception as e:
                self.get_logger().error(f'STM32 read error: {e}')
                break
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
                raw = self.arduino_ser.readline()
                if raw:
                    line = raw.decode('utf-8', errors='replace').rstrip('\r\n')
                    if line:
                        yaw_deg = self._parse_yaw(line)
                        if yaw_deg is not None:
                            # ✅ KONVERSI DERAJAT → RADIAN
                            yaw_rad = math.radians(yaw_deg)
                            self.current_yaw = self._normalize_angle(yaw_rad)
                            self._publish_imu(self.current_yaw)
                            self.get_logger().debug(f'[Arduino] Yaw: {yaw_deg:.2f}° = {self.current_yaw:.3f} rad')
                else:
                    time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Arduino read error: {e}')
                break
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

    # ========== MAIN LOOP: Publish Odom Periodically ==========
    def spin_loop(self):
        """Main loop: publish odometry at fixed rate."""
        rate = self.create_rate(50)  # 50 Hz odom publish
        while rclpy.ok():
            self._update_and_publish_odom()
            rate.sleep()

    def destroy_node(self):
        """Cleanup serial connections."""
        if self.stm32_ser:
            self.stm32_ser.close()
        if self.arduino_ser:
            self.arduino_ser.close()
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