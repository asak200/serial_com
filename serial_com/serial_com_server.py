#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.msg import SerMsg
from pose_int.srv import CmdVelReq
from example_interfaces.msg import String

import serial, time, threading

class ComNode(Node):

    def __init__(self):
        super().__init__('serial_com_node')
        self.pub = self.create_publisher(SerMsg, 'serial_read', 10)
        self.pub_enc = self.create_publisher(SerMsg, 'enc_val', 10)

        self.ser_port = f'/dev/ttyUSB0'
        self.ser = serial.Serial(self.ser_port, 115200, timeout=1.0)
        # for i in range(10):
        #     try:
        #         self.ser_port = f'/dev/ttyUSB{i}'
        #         self.ser = serial.Serial(self.ser_port, 115200, timeout=1.0)
        #         break
        #     except serial.serialutil.SerialException:
        #         pass

        time.sleep(2.)
        self.ser.reset_input_buffer()
        self.get_logger().info(f"Serial com established to {self.ser_port}")

        self.vel_srv = self.create_service(CmdVelReq, 'send_vel_srv', self.send_vel)
        self.el = '0'
        self.er = '0'

        self.serial_thread = threading.Thread(target=self.listen)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # initialize
        msg = SerMsg()
        msg.head = 'enc'
        msg.info = '0  0  0  0  0'
        for _ in range(2000):
            self.pub_enc.publish(msg)

    def listen(self):
        self.get_logger().info("listen")
        try:
            self.get_logger().info("try")
            while True:
                time.sleep(0.01)
                # self.get_logger().info("while")
                if self.ser.in_waiting > 0: # to receive 
                    # self.get_logger().info("if")
                    line = self.ser.readline().decode('utf-8').rstrip()
                    # self.get_logger().info("read")
                    self.get_logger().info(line)
                    self.analize_msg(line)
                    # self.ser.reset_input_buffer()

        except KeyboardInterrupt:
            print('close serial')
            self.get_logger().info("close")
            self.ser.close()
        self.get_logger().info("done")
    
    def analize_msg(self, line: str):
        if not ': ' in line or len(line.split(': ')) != 2:
            return
        order, content = line.split(': ')
        msg = SerMsg()
        msg.head = order
        msg.info = content
        if order == 'enc':
            self.pub_enc.publish(msg)
            c = content.split("  ")
            self.get_logger().info(f"{c}")
        else:
            self.pub.publish(msg)
            # self.get_logger().info(f"{msg.info}")


    def send_vel(self, req: CmdVelReq.Request, resp):
        msg = req.speed_request
        # self.get_logger().info(msg)
        self.ser.write(msg.encode('utf-8'))
        # self.get_logger().info(f"sending: {msg[3:-1]}")
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ComNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
