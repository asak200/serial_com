#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.msg import SerMsg
from pose_int.srv import CmdVelReq
from geometry_msgs.msg import TransformStamped, Twist
from tf2_msgs.msg import TFMessage
from example_interfaces.msg import Float64

from transforms3d.euler import euler2quat
from math import pi, cos, sin

class DiffContNode(Node):
    def __init__(self):
        super().__init__('diff_cont_asak')
        self.enc_listener = self.create_subscription(SerMsg, 'enc_val', self.get_enc, 10)
        self.joy_listener = self.create_subscription(Twist, 'cmd_vel_joy', self.apply_vel, 10)
        self.vel_cli = self.create_client(CmdVelReq, 'send_vel_srv')
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 10)
        self.publish_vel = self.create_publisher(Float64, 'feedback_speed', 10)

        self.my_timer = self.create_timer(0.02, self.update_pose_vel)

        self.xl = 0
        self.xr = 0
        self.l = 0
        self.r = 0
        self.pose_x = 0
        self.pose_y = 0
        self.pose_ang = 0
        self.prev_enc_l = 0
        self.prev_enc_r = 0
        self.prev_now = self.get_clock().now().nanoseconds

        # small wheel
        self.ENC_COUNT_PER_REV = 22
        self.radius = 0.035
        self.wheel_separation = 0.14
        # real wheel
        # self.ENC_COUNT_PER_REV = 600
        # self.radius = 0.1
        # self.wheel_separation = 0.55

        self.req = CmdVelReq.Request()
        while not self.vel_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info("Diff drive controller initialized")


    def get_enc(self, enc_info: SerMsg):
        if not '   ' in enc_info.info or len(enc_info.info.split('   ')) != 2:
            return
        l, r = enc_info.info.split('   ')
        if l == '' or r == '':
            return
        self.l = l
        self.r = r
        
        # print(f"x: {self.pose_x}\ny: {self.pose_y}\nang: {self.pose_ang}")

    def update_pose_vel(self):
        denc_l = int(self.l) - self.prev_enc_l
        denc_r = int(self.r) - self.prev_enc_r
        dxl = denc_l*2*pi*self.radius / self.ENC_COUNT_PER_REV
        dxr = denc_r*2*pi*self.radius / self.ENC_COUNT_PER_REV
        self.prev_enc_l = int(self.l)
        self.prev_enc_r = int(self.r)

        d = (dxr+dxl)/2
        ang = (dxr-dxl)/self.wheel_separation

        self.pose_x += d * cos(self.pose_ang + ang/2)
        self.pose_y += d * sin(self.pose_ang + ang/2)
        self.pose_ang += ang

        self.xl += dxl
        self.xr += dxr
        now = self.get_clock().now().nanoseconds
        dt = now - self.prev_now

        self.real_vl = dxl/dt * 10**9
        self.real_vr = dxr/dt * 10**9

        msg = Float64()
        msg.data = self.real_vl/2 + self.real_vr/2
        self.publish_vel.publish(msg)
        self.pulish_to_tf()

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
        
    def apply_vel(self, msg: Twist):
        vx = msg.linear.x
        az = msg.angular.z
        vl = vx - az*self.wheel_separation/2
        vr = vx + az*self.wheel_separation/2
        pwm_l = 150
        pwm_r = 150
        self.req.speed_request = f"vs: {pwm_l} {pwm_r}\n"
        if vx and az:
            pass
        elif vx:
            if vx < 0:
                pwm_l = -150
                pwm_r = -150
                self.req.speed_request = f"vs:{pwm_l}{pwm_r}\n"
        elif az:
            if az > 0:
                pwm_l = -150
                self.req.speed_request = f"vs:{pwm_l} {pwm_r}\n"
            else:
                pwm_r = -150
                self.req.speed_request = f"vs: {pwm_l}{pwm_r}\n"
        else:
            pwm_l = 0
            pwm_r = 0
            self.req.speed_request = "vs: 000 000\n"
        
        # self.get_logger().info("sendin")
        self.vel_cli.call_async(self.req)

        

def main(args=None):
    rclpy.init(args=args)
    node = DiffContNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()