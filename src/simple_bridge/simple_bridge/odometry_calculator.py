# /slam_ws/ws/src/simple_bridge/simple_bridge/odometry_calculator.py
import math
import json
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class OdometryCalculator:
    """
    Handle odometry calculation and config loading.
    Returns calculated values, does NOT publish directly.
    """
    
    def __init__(self, config_path=None, node=None):
        """
        Initialize calculator and load config.
        
        Args:
            config_path: Path to bridge_params.json
            node: ROS2 node (for logging, optional)
        """
        self.node = node
        self.config_path = config_path or os.path.expanduser(
            '/slam_ws/ws/src/simple_bridge/config/bridge_params.json'
        )
        
        # State
        self.x = 0.0
        self.y = 0.0
        self.last_time = None
        
        # Config (loaded in constructor)
        self.odom_scale_linear = 1.0
        self.odom_scale_angular = 1.0
        self._load_config()
    
    def _load_config(self):
        """Load calibration factors from JSON config."""
        try:
            with open(self.config_path, 'r') as f:
                params = json.load(f)
                self.odom_scale_linear = params.get('odom_scale_linear', 1.0)
                self.odom_scale_angular = params.get('odom_scale_angular', 1.0)
                if self.node:
                    self.node.get_logger().info(
                        f'Loaded params: linear_scale={self.odom_scale_linear}, '
                        f'angular_scale={self.odom_scale_angular}'
                    )
        except Exception as e:
            if self.node:
                self.node.get_logger().warn(f'Gagal load params: {e}. Menggunakan default scale=1.0')
    
    def reload_config(self):
        """Reload config from file (call when file changes)."""
        self._load_config()
    
    def update(self, twist, imu_theta, current_time):
        """
        Calculate odometry from sensor data.
        
        Args:
            twist: dict with 'linear_x' and 'angular_z'
            imu_theta: yaw orientation (radian, relatif dari start)
            current_time: ROS2 Time object
        
        Returns:
            dict with:
                - x, y: position
                - theta: orientation
                - linear_x, angular_z: twist (scaled)
                - odom_msg: Odometry message (ready to publish)
                - tf_msg: TransformStamped (ready to publish)
        """
        # Initialize last_time on first call
        if self.last_time is None:
            self.last_time = current_time
        
        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Always update last_time
        self.last_time = current_time
        
        # Integrate position (only if dt valid)
        if dt > 0 and dt < 1.0:
            vx_raw = twist.get('linear_x', 0.0)
            wz_raw = twist.get('angular_z', 0.0)
            
            # Apply calibration scales
            vx = vx_raw * self.odom_scale_linear
            wz = wz_raw * self.odom_scale_angular
            
            # Integrate
            self.x += vx * math.cos(imu_theta) * dt
            self.y += vx * math.sin(imu_theta) * dt
        
        # Build Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(imu_theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(imu_theta / 2.0)
        odom_msg.twist.twist.linear.x = twist.get('linear_x', 0.0) * self.odom_scale_linear
        odom_msg.twist.twist.angular.z = twist.get('angular_z', 0.0) * self.odom_scale_angular
        
        # Build TF message
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = math.sin(imu_theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(imu_theta / 2.0)
        
        # Return all calculated values
        return {
            'x': self.x,
            'y': self.y,
            'theta': imu_theta,
            'linear_x': odom_msg.twist.twist.linear.x,
            'angular_z': odom_msg.twist.twist.angular.z,
            'odom_msg': odom_msg,
            'tf_msg': tf_msg
        }
    
    def reset(self):
        """Reset odometry to (0, 0, 0)."""
        self.x = 0.0
        self.y = 0.0
        self.last_time = None
        if self.node:
            self.node.get_logger().info('Odometry reset to (0.0, 0.0, 0.0)')