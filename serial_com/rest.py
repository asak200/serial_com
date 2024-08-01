#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from math import pi

class JointBroad(Node):
    def __init__(self):
        super().__init__('joint_broad_asak')
        self.my_timer = self.create_timer(0.02, self.update_pose_vel)
        self.prev_now = self.get_clock().now().nanoseconds
        
    def update_pose_vel(self):
        now = self.get_clock().now().nanoseconds
        self.get_logger().info(f"{now - self.prev_now}")
        self.prev_now = now

def main(args=None):
    rclpy.init(args=args)
    node = JointBroad()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()