import rclpy

from rclpy.node import Node
from sensor_msgs.msg import Imu

from serial import Serial
import numpy as np

class WT901BLECL(Node):


    def __init__(self):
        super().__init__('wt901_driver')
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.myserial = Serial("/dev/ttyUSB1", baudrate = 115200, timeout = 1)
        self.myserial.reset_input_buffer()
        self.myserial.write([0xFF,0xAA,0x03,0x08,0x00]) # 50Hz
        self.myserial.write([0xFF,0xAA,0x00,0x00,0x00]) # SAVE
        
    def timer_callback(self):
        data = self.myserial.read(size=20)
        if not len(data) == 20:
            self.get_logger().info('Byte Error'+str(len(data)))
        elif( (data[0] == 0x55) and (data[1] == 0x61) ):

            #Acceleration
            self.accel_x = int.from_bytes(data[2:4], byteorder='little', signed="True")/32768.0*16
            self.accel_y = int.from_bytes(data[4:6], byteorder='little', signed="True")/32768.0*16
            self.accel_z = int.from_bytes(data[6:8], byteorder='little', signed="True")/32768.0*16

            #Angular velocity
            self.angular_velocity_x = int.from_bytes(data[8:10], byteorder='little', signed="True")/32768*2000
            self.angular_velocity_y = int.from_bytes(data[10:12], byteorder='little', signed="True")/32768*2000
            self.angular_velocity_z = int.from_bytes(data[12:14], byteorder='little', signed="True")/32768*2000

            #Angle (deg)
            self.roll_deg = int.from_bytes(data[14:16], byteorder='little', signed="True")/32768*180
            self.pitch_deg = int.from_bytes(data[16:18], byteorder='little', signed="True")/32768*180
            self.yaw_deg = int.from_bytes(data[18:20], byteorder='little', signed="True")/32768*180

            # Attitude (rad)
            roll  = self.roll_deg * 0.0174533 # convert to rad
            pitch = self.pitch_deg * 0.0174533 # convert to rad
            yaw   = self.yaw_deg * 0.0174533 # convert to rad

            # euler to quaternion
            # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
            cr = np.cos(roll * 0.5);
            sr = np.sin(roll * 0.5);
            cp = np.cos(pitch * 0.5);
            sp = np.sin(pitch * 0.5);
            cy = np.cos(yaw * 0.5);
            sy = np.sin(yaw * 0.5);
            qw = cr * cp * cy + sr * sp * sy;
            qx = sr * cp * cy - cr * sp * sy;
            qy = cr * sp * cy + sr * cp * sy;
            qz = cr * cp * sy - sr * sp * cy;

            msg = Imu()
            msg.header.frame_id = "imu"
            msg.header.stamp = super().get_clock().now().to_msg()
            msg.linear_acceleration.x = self.accel_x * 9.80665 # convert to m/s²
            msg.linear_acceleration.y = self.accel_y * 9.80665 # convert to m/s² 
            msg.linear_acceleration.z = self.accel_z * 9.80665 # convert to m/s²
            msg.linear_acceleration_covariance[0] = 0.005 * 9.80665 # convert to m/s²
            msg.linear_acceleration_covariance[4] = 0.005 * 9.80665 # convert to m/s²
            msg.linear_acceleration_covariance[8] = 0.005 * 9.80665 # convert to m/s²
            msg.angular_velocity.x = self.angular_velocity_x * 0.0174533 # convert to rad/s
            msg.angular_velocity.y = self.angular_velocity_y * 0.0174533 # convert to rad/s
            msg.angular_velocity.z = self.angular_velocity_z * 0.0174533 # convert to rad/s
            msg.angular_velocity_covariance[0] = 0.05 * 0.0174533 # convert to rad/s
            msg.angular_velocity_covariance[4] = 0.05 * 0.0174533 # convert to rad/s
            msg.angular_velocity_covariance[8] = 0.05 * 0.0174533 # convert to rad/s
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
            msg.orientation_covariance[0] = 0.05 * 0.0174533 # convert to rad
            msg.orientation_covariance[4] = 0.05 * 0.0174533 # convert to rad
            msg.orientation_covariance[8] = 0.05 * 0.0174533 # convert to rad
            self.publisher_.publish(msg)

            #self.get_logger().info('Yaw:'+str(self.yaw_deg))  
        else:
            self.get_logger().info('Frame Error')

def main(args=None):
    rclpy.init(args=args)
    imu = WT901BLECL()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

