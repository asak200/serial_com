#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.msg import SerMsg
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from transforms3d.euler import euler2quat

from math import pi, cos, sin

class DiffContNode(Node):
    def __init__(self):
        super().__init__('diff_cont_asak')
        self.enc_listener = self.create_subscription(
            SerMsg,
            'enc_val',
            self.get_enc,
            10
        )
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 10)
        self.tf_pub
        self.pose_x = 0
        self.pose_y = 0
        self.pose_ang = 0
        self.ENC_COUNT_PER_REV = 20
        self.radius = 0.1
        self.wheel_separation = 0.57
        self.prev_enc_l = 0
        self.prev_enc_r = 0

        self.get_logger().info("Diff drive controller initialized")


    def get_enc(self, enc_info: SerMsg):
        if not '   ' in enc_info.info or len(enc_info.info.split('   ')) != 2:
            return
        l, r = enc_info.info.split('   ')
        if l == '' or r == '':
            return
        denc_l = int(l) - self.prev_enc_l
        denc_r = int(r) - self.prev_enc_r
        xl = denc_l*2*pi*self.radius / self.ENC_COUNT_PER_REV
        xr = denc_r*2*pi*self.radius / self.ENC_COUNT_PER_REV
        self.prev_enc_l = int(l)
        self.prev_enc_r = int(r)
        
        d = (xr+xl)/2
        ang = (xr-xl)/self.wheel_separation

        self.pose_x += d * cos(self.pose_ang + ang/2)
        self.pose_y += d * sin(self.pose_ang + ang/2)
        self.pose_ang += ang

        self.pulish_to_tf()
        # print(f"x: {self.pose_x}\ny: {self.pose_y}\nang: {self.pose_ang}")
        
    def pulish_to_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose_x 
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        quat = euler2quat(0, 0, self.pose_ang)
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        t.transform.rotation.w = quat[0]

        msg = TFMessage(transforms=[t])
        self.tf_pub.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = DiffContNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()