#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Cmd Publisher siap')

    def timer_callback(self):
        msg = Twist()
        if self.counter % 2 == 0:
            msg.linear.x = 0.5
            msg.angular.z = 0.0
        else:
            msg.linear.x = 0.5
            msg.angular.z = 0.3
        self.pub.publish(msg)
        self.get_logger().info(f'Mengirim: linear={msg.linear.x}, angular={msg.angular.z}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CmdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()