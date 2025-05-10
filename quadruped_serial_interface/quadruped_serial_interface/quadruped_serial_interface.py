# Copyright 2025 Jackson Huang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#！/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from serial import Serial
import time

class QuadrupedSerialInterface(Node):
    
    def __init__(self):
        super().__init__('quadruped_serial_interface')
        self.get_logger().info('Quadruped Serial Interface Node Started')

        # 初始化Serial
        self.serial_port = '/dev/ttyUSB0'
        self.baud_rate = 115200
        self.time_sleep = 0.1
        self.serial= Serial(port = self.serial_port, baudrate = self.baud_rate, timeout = self.time_sleep)

        self.serial_cmd_sub = self.create_subscription(
            String,
            '/serial_cmd',
            self.serial_command_callback,
            20
        )

    def serial_command_callback(self, cmd_msg):
        self.get_logger().info(cmd_msg.data)
        if  (cmd_msg.data == "1"):
            self.serial.write("s100000000000000000e\r\n".encode())
        elif(cmd_msg.data == "2"):
            self.serial.write("s010000000000000000e\r\n".encode())
        elif(cmd_msg.data == "3"):
            self.serial.write("s001000000000000000e\r\n".encode())
        elif(cmd_msg.data == "4"):
            self.serial.write("s000100000000000000e\r\n".encode())
        elif(cmd_msg.data == "5"):
            self.serial.write("s000010000000000000e\r\n".encode())
        elif(cmd_msg.data == "6"):
            self.serial.write("s000001000000000000e\r\n".encode())
        elif(cmd_msg.data == "7"):
            self.serial.write("s000000100000000000e\r\n".encode())
        elif(cmd_msg.data == "8"):
            self.serial.write("s000000010000000000e\r\n".encode())
        elif(cmd_msg.data == "9"):
            self.serial.write("s000000001000000000e\r\n".encode())
        elif(cmd_msg.data == "10"):
            self.serial.write("s000000000100000000e\r\n".encode())
        elif(cmd_msg.data == "11"):
            self.serial.write("s000000000010000000e\r\n".encode())
        elif(cmd_msg.data == "12"):
            self.serial.write("s000000000001000000e\r\n".encode())
        elif(cmd_msg.data == "13"):
            self.serial.write("s000000000000100000e\r\n".encode())
        elif(cmd_msg.data == "14"):
            self.serial.write("s000000000000010000e\r\n".encode())
        elif(cmd_msg.data == "15"):
            self.serial.write("s000000000000001000e\r\n".encode())
        elif(cmd_msg.data == "16"):
            self.serial.write("s000000000000000100e\r\n".encode())
        elif(cmd_msg.data == "17"):
            self.serial.write("s000000000000000010e\r\n".encode())
        elif(cmd_msg.data == "18"):
            self.serial.write("s000000000000000001e\r\n".encode())
        else:
            self.get_logger().info('Invalid Command')   


def main(args=None):
    rclpy.init(args=args)
    serial_interface = QuadrupedSerialInterface()
    rclpy.spin(serial_interface)
    serial_interface.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()