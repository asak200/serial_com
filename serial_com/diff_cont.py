#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.msg import SerMsg
from pose_int.srv import CmdVelReq
from geometry_msgs.msg import TransformStamped, Twist
from tf2_msgs.msg import TFMessage
from example_interfaces.msg import Float64, String
from sensor_msgs.msg import Joy

import time
from transforms3d.euler import euler2quat
from math import pi, cos, sin

MAXSPEED = 0.54
MINSPEED = 0.10

class DiffContNode(Node):
    def __init__(self):
        super().__init__('diff_cont_asak')
        self.enc_listener = self.create_subscription(SerMsg, 'enc_val', self.get_enc, 10)
        # self.joy_listener = self.create_subscription(Twist, 'cmd_vel', self.apply_constant_vel, 10)
        self.joy_listener = self.create_subscription(Twist, 'cmd_vel', self.asak_control_vel, 10)
        self.joy_but = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.vel_cli = self.create_client(CmdVelReq, 'send_vel_srv')
        self.tf_pub = self.create_publisher(TFMessage, 'tf', 10)
        self.publish_vel = self.create_publisher(String, 'speed_feedback', 10)
        self.publish_time = self.create_publisher(Float64, 'time_feedback', 10)

        # self.my_timer = self.create_timer(0.01, self.update_pose)
        # self.my_timer_ = self.create_timer(0.02, self.apply_gui_vel)
        # self.req.speed_request = 'or: c\n'
        # self.vel_cli.call_async(self.req)
        
        self.xl = 0
        self.xr = 0
        self.dxl = 0
        self.dxr = 0
        self.l = 0
        self.r = 0
        self.pose_x = 0.
        self.pose_y = 0.
        self.pose_ang = 0.
        self.real_vl = 0
        self.real_vr = 0

        self.up_down = 0

        self.wheel_separation = 0.55

        self.req = CmdVelReq.Request()
        # while not self.vel_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting...')
        self.get_logger().info("Diff drive controller initialized")

    def joy_callback(self, msg:Joy):
        # self.prev_joyA = self.joyA
        self.joyX = msg.buttons[2]
        self.joyY = msg.buttons[3]
        
        req = CmdVelReq.Request

        if self.joyX:
            self.up_down = 1
            cmd = f'ac: 1\n'
            req.speed_request = cmd
            self.get_logger().info(cmd)
            self.vel_cli.call_async(self.req)
        elif self.joyY:
            self.up_down = 1
            cmd = f'ac: 2\n'
            req.speed_request = cmd
            self.get_logger().info(cmd)
            self.vel_cli.call_async(self.req)
        elif self.up_down:
            self.up_down = 0
            for _ in range(5):
                cmd = f'ac: 0\n'
                req.speed_request = cmd
                self.get_logger().info(cmd)
                self.vel_cli.call_async(self.req)

    def get_enc(self, enc_info: SerMsg):
        if not ' ' in enc_info.info or len(enc_info.info.split(' ')) != 6:
            return
        dxl, dxr, vl, vr, sl, sr = enc_info.info.split(' ')
        if dxl == '' or dxr == '' or vl == '' or vr == '':
            return
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()

        self.dxl = float(dxl)
        self.dxr = float(dxr)
        self.real_vl = float(vl)
        self.real_vr = float(vr)
        # self.xl += self.dxl
        # self.xr += self.dxr
        
        d = (self.dxr+self.dxl)/2
        ang = (self.dxr-self.dxl)/self.wheel_separation

        self.pose_x += d * cos(self.pose_ang + ang/2)
        self.pose_y += d * sin(self.pose_ang + ang/2)
        self.pose_ang += ang
        # self.get_logger().info(f'{self.xl} {self.xr} '+ ' ' + xl + ' ' + xr)
        self.update_pose(tf)

    def update_pose(self, tf: TransformStamped):
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.pose_x
        tf.transform.translation.y = self.pose_y
        tf.transform.translation.z = 0.0
        quat = euler2quat(0, 0, self.pose_ang)
        tf.transform.rotation.w = quat[0]
        tf.transform.rotation.x = 0.
        tf.transform.rotation.y = 0.
        tf.transform.rotation.z = quat[3]

        msg = TFMessage(transforms=[tf])
        self.tf_pub.publish(msg)
        
    def asak_control_vel(self, msg: Twist):
        vx = msg.linear.x
        az = msg.angular.z
        vl = vx - az*self.wheel_separation/2
        vr = vx + az*self.wheel_separation/2

        def constrain(x):
            def sign(n):
                if n>=0:
                    return 1
                else:
                    return -1
            if abs(x) > MAXSPEED:
                x =  MAXSPEED * sign(x)
            elif 0.05 < abs(x) < MINSPEED:
                x = MINSPEED * sign(x)
            elif 0.05 >= abs(x):
                x = 0.
            return x

        vl = constrain(vl)
        vr = constrain(vr)
        cmd = f'vs:{vl: 0.2f}{vr: 0.2f}\n'
        req = CmdVelReq.Request
        req.speed_request = cmd
        self.get_logger().info(cmd)
        self.vel_cli.call_async(self.req)

    def apply_constant_vel(self, msg: Twist):
        vx = msg.linear.x
        az = msg.angular.z
        pwm_l = 200
        pwm_r = 200

        cmd = f"vs: {pwm_l} {pwm_r}"
        if vx == 0. and az == 0.:
            pwm_l = 0
            pwm_r = 0
            cmd = "vs: 000 000"
        elif vx < 0:
            if vx < 0:
                pwm_l *= -1
                pwm_r *= -1
                cmd = f"vs:{pwm_l}{pwm_r}"
        elif az > 0 and vx == 0:
            pwm_l = -120
            pwm_r = 120
            cmd = f"vs:{pwm_l} {pwm_r}"
        elif az < 0 and vx == 0:
            pwm_l = 120
            pwm_r = -120
            cmd = f"vs: {pwm_l}{pwm_r}"
        
        self.req.speed_request = cmd + '\n'
        # self.get_logger().info(f"sendin {}, {az}")
        self.vel_cli.call_async(self.req)

    def apply_gui_vel(self):
        with open("/home/asak/dev_ws2/src/cmd.txt", 'r') as file:
            v = file.read()
        
        cmd = f"vs: 0.40 0.40"
        if v == 's':
            pwm_l = 0
            pwm_r = 0
            cmd = "vs: 0.00 0.00"
        elif v == 'b':
            pwm_l *= -1
            pwm_r *= -1
            cmd = f"vs:-0.40-0.40"
        elif v == 'l':
            pwm_l = -120
            pwm_r = 120
            cmd = f"vs:-0.40 0.40"
        elif v == 'r':
            pwm_l = 120
            pwm_r = -120
            cmd = f"vs: 0.40-0.40"
        
        self.req.speed_request = cmd + '\n'
        self.get_logger().info(f"sendin {self.req.speed_request}")
        self.vel_cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = DiffContNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()