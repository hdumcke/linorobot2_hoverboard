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

import rclpy
import traceback
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hoverboard_controller.hoverboard_controller import HoverboardInterface as HI


class HoverboardInterface(Node):
    def __init__(self):
        super().__init__('hoverboard_interface')
        self.ct = HI()
        self.subscriber = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_callback, 1)

    def cmd_callback(self, vel_cmd):
        # angular_vel_deg = vel_cmd.angular.z * 360.0 / (2.0 * np.pi)
        # linear_velocity = vel_cmd.linear.x
        #TODO go slow during testing, remove later
        # self.ct.set_speed(int(2000 * vel_cmd.linear.x), int(1000 * vel_cmd.angular.z))
        self.ct.set_speed(int(200 * vel_cmd.linear.x), int(1000 * vel_cmd.angular.z))


def main(args=None):
    rclpy.init(args=args)
    hoverboard_interface_node = HoverboardInterface()
    try:
        rclpy.spin(hoverboard_interface_node)
    except Exception as exception:
        traceback_logger_node = Node('node_class_traceback_logger')
        traceback_logger_node.get_logger().error(traceback.format_exc())
        raise exception
    else:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
