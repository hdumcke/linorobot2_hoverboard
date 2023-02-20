import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from hoverboard_controller.hoverboard_controller import HoverboardInterface

import numpy as np
import quaternion
import simple_pid


class Horo(Node):

    def __init__(self):
        super().__init__('horo_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = TransformBroadcaster(self, 10) #odom frame broadcaster
        self.r = 0.0825
        self.b = 2. * 0.18
        self.a_limit = 10.0
        self.ct_l = simple_pid.PID(Kp=.6, Ki=0., Kd=.0, sample_time=None)
        self.ct_r = simple_pid.PID(Kp=.6, Ki=0., Kd=.0, sample_time=None)
        self.error_l = 0.0
        self.error_r = 0.0
        self.previous = {}
        self.previous["linear"] = 0.
        self.previous["angular"] = 0.
        self.dt = 1./20.
        self.horo = HoverboardInterface()
        self.time_last = None
        self.time_now = None
        self.x0 = 0
        self.y0 = 0
        self.Theta0 = 0
        # calibration
        self.cal_r = 100. / 14.65
        self.cal_l = 100. / 20.36 

    def update_previous(self, l_x, w):
        self.previous["linear"] = l_x
        self.previous["angular"] = w

    def get_wheel_speed(self, l_x, w):
        w_r = (l_x + (self.b/2) * w) / self.r
        w_l = (l_x - (self.b/2) * w) / self.r
        return w_l, w_r

    def get_pose(self, x0, y0, Theta0, v_r, v_l, delta_t):
        Theta1 = Theta0 + delta_t * (v_r - v_l)
        x1 = x0 + delta_t * (v_r + v_l) * np.cos(Theta1/2)
        y1 = y0 + delta_t * (v_r + v_l) * np.sin(Theta1/2)
        q1 = np.quaternion(np.cos(Theta1/2), 0, 0, np.sin(Theta1/2))
        return x1, y1, Theta1, q1

    def listener_callback(self, msg):
        if self.time_last is None:
            self.time_last = self.get_clock().now().nanoseconds
            return
        self.time_now = self.get_clock().now().nanoseconds
        self.dt = (self.time_now - self.time_last) / 1e9
        self.time_last = self.time_now
        v_x = msg.linear.x
        v_z = msg.angular.z
        self.update_previous(v_x, v_z)
        w_l, w_r = self.get_wheel_speed(v_x, v_z)
        #self.get_logger().info('msg: [%.2f, %.2f]   wheel: [%.2f, %.2f]' % (v_x, v_z, w_l, w_r))
        cr_l = self.ct_l(self.error_l)
        cr_r = self.ct_r(self.error_r)
        set_pos_r = w_r + cr_r
        set_pos_l = w_l + cr_l
        if w_r>= 50. and set_pos_r >= 0. and set_pos_r < 50.:
            set_pos_r = 50
        if w_r<= -50. and set_pos_r <= 0. and set_pos_r > -50.:
            set_pos_r = -50
        if w_l>= 50. and set_pos_l >= 0. and set_pos_l < 50.:
            set_pos_l = 50
        if w_l<= -50. and set_pos_l <= 0. and set_pos_l > -50.:
            set_pos_l = -50
        self.get_logger().info('msg: [%.2f, %.2f] controller: [%.2f, %.2f]   wheel: [%.2f, %.2f] set: [%s, %s]' % (v_x, v_z, cr_l,  cr_r, w_l, w_r, int(set_pos_l), int(set_pos_r)))
        self.horo.set_speed(int(set_pos_l), int(set_pos_r))
        v_r, v_l = self.horo.get_speed()
        self.error_l = w_l - self.cal_l * v_l
        self.error_r = w_r - self.cal_r * v_r
        x1, y1, Theta1, q1 = self.get_pose(self.x0, self.y0, self.Theta0, v_r, v_l, self.dt)
        self.x0 = x1
        self.y0 = y1
        self.Theta0 = Theta1
        v_x = (v_r + v_l) / 2
        a_z = (v_r - v_l) / self.b
        self.publish_odometry(v_x, a_z, self.x0, self.y0, 0., q1.x, q1.y, q1.z, q1.w)

    def publish_odometry(self, v_x, a_z, x, y, z, quat_x, quat_y, quat_z, quat_w):
        msg = Odometry()
        current_time = self.get_clock().now().to_msg()
        msg.header.stamp = current_time
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat_x
        msg.pose.pose.orientation.y = quat_y
        msg.pose.pose.orientation.z = quat_z
        msg.pose.pose.orientation.w = quat_w
        msg.twist.twist.linear.x = v_x
        msg.twist.twist.linear.y = 0.
        msg.twist.twist.linear.z = 0.
        msg.twist.twist.angular.x = 0.
        msg.twist.twist.angular.y = 0.
        msg.twist.twist.angular.z = a_z
        self.publisher.publish(msg)

        tfs = TransformStamped()
        tfs.header.stamp = current_time
        tfs.header.frame_id = 'world'
        tfs.child_frame_id = 'base_link'
        tfs.transform.translation.x = msg.pose.pose.position.x
        tfs.transform.translation.y = msg.pose.pose.position.y
        tfs.transform.translation.z = msg.pose.pose.position.z
        tfs.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(tfs)
        tfs.child_frame_id = 'base_footprint'
        self.broadcaster.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)

    horo = Horo()

    rclpy.spin(horo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    horo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
