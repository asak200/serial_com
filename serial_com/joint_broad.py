#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pose_int.msg import SerMsg
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


from math import pi

class JointBroad(Node):
    def __init__(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        super().__init__('joint_broad_asak')
        self.enc_listener = self.create_subscription(SerMsg, 
            'enc_val', self.get_enc, qos_profile
        )
        self.joint_states_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        self.my_timer = self.create_timer(0.01, self.update_pose_vel)

        self.joint_state_msg = JointState()
        self.ENC_COUNT_PER_REV = 2400
        self.con = 2*pi / self.ENC_COUNT_PER_REV

        self.get_logger().info("joint state broadcaster initialized")

    def get_enc(self, enc_info: SerMsg):
        if not '   ' in enc_info.info or len(enc_info.info.split('   ')) != 2:
            return
        l, r = enc_info.info.split('   ')
        if l == '' or r == '':
            return
        self.l = int(l)
        self.r = int(r)
        
    def publish_jt(self):
        pl = self.l*self.con
        pr = self.r*self.con
        
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