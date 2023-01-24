#!/usr/bin/env python3

# MIT License
#
# Copyright (c) 2022 MangDang
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# @Author  : Zhengxiao Han

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class HoverboardInterface(Node):
    def __init__(self):
        super().__init__('hoverboard_interface')
        self.subscriber = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_callback, 1)

    def cmd_callback(self, vel_cmd):
        angular_vel_deg = vel_cmd.angular.z * 360.0 / (2.0 * np.pi)
        linear_velocity = vel_cmd.linear.x


def main(args=None):
    rclpy.init(args=args)
    hoverboard_interface_node = HoverboardInterface()
    rclpy.spin(hoverboard_interface_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
