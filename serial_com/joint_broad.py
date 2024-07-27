#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pose_int.msg import SerMsg

from math import pi

class JointBroad(Node):
    def __init__(self):
        super().__init__('joint_broad_asak')
        self.enc_listener = self.create_subscription(
            SerMsg,
            'enc_val',
            self.get_enc,
            10
        )
        self.joint_states_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        self.joint_state_msg = JointState()
        self.ENC_COUNT_PER_REV = 3000

        self.get_logger().info("joint state broadcaster initialized")

    def get_enc(self, enc_info: SerMsg):
        if not '   ' in enc_info.info or len(enc_info.info.split('   ')) != 2:
            return
        l, r = enc_info.info.split('   ')
        pl = int(l)*2*pi / self.ENC_COUNT_PER_REV
        pr = int(r)*2*pi / self.ENC_COUNT_PER_REV
        
        # Update joint states
        self.joint_state_msg.header = Header()
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.header.frame_id = ""

        self.joint_state_msg.name = ["left_wheel_joint", "right_wheel_joint"]
        self.joint_state_msg.position = [pl, pr]

        # Publish the message
        self.joint_states_pub.publish(self.joint_state_msg)



def main(args=None):
    rclpy.init(args=args)
    node = JointBroad()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()