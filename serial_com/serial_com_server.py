#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.msg import SerMsg
from pose_int.srv import CmdVelReq

import serial, time, threading

class ComNode(Node):

    def __init__(self):
        super().__init__('serial_com_node')

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

        self.pub = self.create_publisher(SerMsg, 'serial_read', 10)
        self.pub_enc = self.create_publisher(SerMsg, 'enc_val', 10)
        self.vel_srv = self.create_service(CmdVelReq, 'send_vel_srv', self.send_vel)
        self.el = '0'
        self.er = '0'
        self.i = 0

        self.serial_thread = threading.Thread(target=self.listen)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        # iniitialize
        msg = SerMsg()
        msg.head = 'enc'
        msg.info = '0   0'
        for _ in range(2000):
            self.pub_enc.publish(msg)

    def listen(self):
        try:
            while True:
                # time.sleep(0.01)
                if self.ser.in_waiting > 0: # to receive 
                    line = self.ser.readline().decode('utf-8').rstrip()
                    self.analize_msg(line)
                # elif self.el == '0' and self.er == '0':
                #     msg = SerMsg()
                #     msg.head = 'enc'
                #     msg.info = '0   0'
                #     self.i += 1
                #     self.pub_enc.publish(msg)
                    
        except KeyboardInterrupt:
            print('close serial')
            self.ser.close()
    
    def analize_msg(self, line: str):
        if not ': ' in line or len(line.split(': ')) != 2:
            return
        order, content = line.split(': ')
        msg = SerMsg()
        msg.head = order
        msg.info = content
        if order == 'enc':
            self.pub_enc.publish(msg)
            if '   ' in content:
                self.el, self.er = content.split('   ')
        else:
            self.pub.publish(msg)

    def send_vel(self, req: CmdVelReq.Request, resp):
        msg = req.speed_request
        # self.get_logger().info(msg)
        self.ser.write(msg.encode('utf-8'))
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = ComNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
