import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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

    def listener_callback(self, msg):
        self.get_logger().info('I heard: (%s, %s)' % (msg.linear.x, msg.angular.z, ))
        self.get_logger().info('I am sending: (%s, %s, %s, %s, %s, %s, %s)' % (0., 0., 0., 0., 0., 0., 1.))
        self.publish_odometry(msg.linear.x, msg.angular.z, 0., 0., 0., 0., 0., 0., 1.)

    def publish_odometry(self, v_x, a_z, x, y, z, quat_x, quat_y, quat_z, quat_w):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
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
