#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "hoverboard-driver-pkg/protocol.hpp"
#include "hoverboard-driver-pkg/pid.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <math.h>

#define PORT "/dev/ttyUSB0"


class driver_node : public rclcpp::Node
{
public:
	driver_node() : Node("hoverboard_driver"),
	_br(this),
	_x_pid(&_x_kp,&_x_ki,&_x_kd,&_x_kff,-1000.0,1000.0),
	_w_pid(&_w_kp,&_w_ki,&_w_kd,&_w_kff,-1000.0,1000.0)
	
	{
		this->declare_parameter("acc",_acceleration);
		this->declare_parameter("dec",_deceleration);

		this->declare_parameter("x_kp",_x_kp);
		this->declare_parameter("x_ki",_x_ki);
        this->declare_parameter("x_kd",_x_kd);
        this->declare_parameter("x_kff",_x_kff);
        this->declare_parameter("x_d_alpha",_x_d_alpha);
        this->declare_parameter("x_o_alpha",_x_o_alpha);

        this->declare_parameter("w_kp",_w_kp);
        this->declare_parameter("w_ki",_w_ki);
        this->declare_parameter("w_kd",_w_kd);
        this->declare_parameter("w_kff",_w_kff);
        this->declare_parameter("w_d_alpha",_w_d_alpha);
        this->declare_parameter("w_o_alpha",_w_o_alpha);

        _acceleration = this->get_parameter("acc").as_double();
        _deceleration = this->get_parameter("dec").as_double();

        _x_kp = this->get_parameter("x_kp").as_double();
        _x_ki = this->get_parameter("x_ki").as_double();
        _x_kd = this->get_parameter("x_kd").as_double();
        _x_kff = this->get_parameter("x_kff").as_double();
        _x_d_alpha = this->get_parameter("x_d_alpha").as_double();
        _x_o_alpha = this->get_parameter("x_o_alpha").as_double();

        _w_kp = this->get_parameter("w_kp").as_double();
        _w_kp = this->get_parameter("w_ki").as_double();
        _w_kd = this->get_parameter("w_kd").as_double();
        _w_kff = this->get_parameter("w_kff").as_double();
        _w_d_alpha = this->get_parameter("w_d_alpha").as_double();
        _w_o_alpha = this->get_parameter("w_o_alpha").as_double();

        _x_pid._d_alpha = _x_d_alpha;
        _x_pid._out_alpha = _x_o_alpha;

        _w_pid._d_alpha = _w_d_alpha;
        _w_pid._out_alpha = _w_o_alpha;

		RCLCPP_INFO(this->get_logger(),"Starting...");
		_cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
			"/cmd_vel",
			10,
			std::bind(&driver_node::_cmd_vel_callback, this, std::placeholders::_1)
		);	

		// odometry publisher
 		_odom_pub = create_publisher<nav_msgs::msg::Odometry>(
 			"/odom",
 			10
 		);

 		_speed_setpoint_pub = create_publisher<geometry_msgs::msg::Twist>(
 			"/speed_setpoint",
 			10
 		);

		// Init serial communication
	    if ((_port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
	        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port to hoverboard");
	        exit(-1); // TODO : put this again
	    }
	    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	    struct termios options;
	    tcgetattr(_port_fd, &options);
	    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	    options.c_iflag = IGNPAR;
	    options.c_oflag = 0;
	    options.c_lflag = 0;
	    tcflush(_port_fd, TCIFLUSH);
	    tcsetattr(_port_fd, TCSANOW, &options);			
	}

	~driver_node()
	{
		if (_port_fd != -1) 
			close(_port_fd);
	}

	void read()
	{
		if(_port_fd != -1)
		{
			uint8_t c;
			int i = 0, r = 0;

			while ((r = ::read(_port_fd, &c, 1)) > 0 && i++ < 1024)
			{
				//RCLCPP_INFO(this->get_logger(), "Reading UART");
				protocol_recv(c);
			}            

			// if (i > 0)
			// last_read = ros::Time::now();

			if (r < 0 && errno != EAGAIN)
				RCLCPP_ERROR(this->get_logger(), "Reading from serial %s failed: %d", PORT, r);
		}

		// if ((ros::Time::now() - last_read).toSec() > 1) {
		//     RCLCPP_ERROR(this->get_logger(), "Timeout reading from serial %s failed", PORT);
		// }
	}

	void protocol_recv (uint8_t byte)
	{
		start_frame = ((uint16_t)(byte) << 8) | prev_byte;
		//RCLCPP_INFO(this->get_logger(), "Received a byte: %x",(uint8_t)byte);
		// if ((uint8_t)byte == 0xAB && (uint8_t)prev_byte == 0xCD){
		//     RCLCPP_INFO(this->get_logger(), "Received Start frame: %x", start_frame);
		//     RCLCPP_INFO(this->get_logger(), "Received Start frame: %x %x", (byte) << 8, (uint8_t)prev_byte);
		// }

		// Read the start frame
		if (start_frame == START_FRAME)
		{
		    //RCLCPP_INFO(this->get_logger(), "Start frame recognised");
		    p = (uint8_t*)&msg;
		    *p++ = prev_byte;
		    *p++ = byte;
		    msg_len = 2;
		} 
		else if (msg_len >= 2 && msg_len < sizeof(serial_feedback))
		{
		    // Otherwise just read the message content until the end
		    *p++ = byte;
		    msg_len++;
		}

		if (msg_len == sizeof(serial_feedback))
		{
		    uint16_t checksum = (uint16_t)(
		        msg.start ^
		        msg.cmd1 ^
		        msg.cmd2 ^
		        msg.right_speed_meas ^
		        msg.left_speed_meas ^
		        msg.voltage ^
		        msg.temperature ^
		        msg.cmd_led);

		    if (msg.start == START_FRAME && msg.checksum == checksum) 
		    {
		    	float voltage_V = (double)msg.voltage/100.0;
		    	float temperature_C = (double)msg.temperature/10.0;
				float left_speed_rpm = -(double)msg.left_speed_meas;
				float right_speed_rpm = (double)msg.right_speed_meas; // need to be inverted into hoverboard firmware

				if(false)
			        RCLCPP_INFO(this->get_logger(), 
			        	"Hoverboard checksum OK: Batt:%.1fV   T:%.1f   Left:%.0fRPM   Right:%0.0fRPM",
			        	voltage_V,
			        	temperature_C,
			        	left_speed_rpm,
			        	right_speed_rpm
			        );

		        // calculate speeds
		        static float const wheel_diameter = 0.165f; 
		        static float const wheel_perimeter = M_PI * wheel_diameter; 
		        float left_speed_mps = left_speed_rpm * wheel_perimeter / 60.0f;
		        float right_speed_mps = right_speed_rpm * wheel_perimeter / 60.0f;

				if(false)
			        RCLCPP_INFO(this->get_logger(), 
			        	"L:%.2fm.s   Right:%.2fm.s",
			        	left_speed_mps,
			        	right_speed_mps
			        );

				// calculate velocities
		        static float const base_width = 0.560f; 
		        _actual_vx = (left_speed_mps+right_speed_mps)/2.0f;
		        _actual_wz = (left_speed_mps-right_speed_mps)/base_width; // approx for small angle

				// calculate distance, angle 
		        static float const dt = 0.010; // 10ms UART feedback rate
		        float dx = _actual_vx*dt;
		        float dh = _actual_wz*dt; // approx for small angle

		        // calculate odometry in Robot Ref Frame, then in World Ref Frame
		        float x_RRF = dx * cos(dh);
		        float y_RRF = dx * sin(dh);
		        _x += x_RRF * cos(_heading) - y_RRF * sin(_heading);
		        _y += x_RRF * sin(_heading) + y_RRF * cos(_heading);
		        _heading += dh;

				if(false)
			        RCLCPP_INFO(this->get_logger(), 
			        	"X:%.2fm   Y:%.2f   H:%.2fdeg",
			        	_x,
			        	_y,
			        	_heading*180.0f/M_PI
			        );

			    // compute quaternion
				tf2::Quaternion q;
				q.setRPY(0.0, 0.0, _heading);

				// get time
				rcutils_time_point_value_t now;
				if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
				{
					RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
				}

		        // publish odom
				auto odom_msg = nav_msgs::msg::Odometry();
				odom_msg.header.frame_id = "odom";
				odom_msg.child_frame_id = "base_link";
				odom_msg.header.stamp.sec = RCL_NS_TO_S(now);
				odom_msg.header.stamp.nanosec = now - RCL_S_TO_NS(odom_msg.header.stamp.sec);

				odom_msg.pose.pose.position.x = _x;
				odom_msg.pose.pose.position.y = _y;
				odom_msg.pose.pose.position.z = 0.0;
			    odom_msg.pose.pose.orientation.x = q.x();
    			odom_msg.pose.pose.orientation.y = q.y();
    			odom_msg.pose.pose.orientation.z = q.z();
    			odom_msg.pose.pose.orientation.w = q.w();

			    for (unsigned int i = 0; i < odom_msg.pose.covariance.size(); ++i)
			    {
      				odom_msg.pose.covariance[i] = 0.0;
    			}

			    // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
			    // Odometry yaw covariance must be much bigger than the covariance provided
			    // by the imu, as the later takes much better measures
			    odom_msg.pose.covariance[0] = 0.1;
			    odom_msg.pose.covariance[7] = 0.1;
			    odom_msg.pose.covariance[35] = 0.2;
			    odom_msg.pose.covariance[14] = 1000.0;  // set a non-zero covariance on unused
			    odom_msg.pose.covariance[21] = 1000.0;  // dimensions (z, pitch and roll); this
			    odom_msg.pose.covariance[28] = 1000.0;  // is a requirement of robot_pose_ekf

			    odom_msg.twist.twist.linear.x = _actual_vx;
			    odom_msg.twist.twist.linear.y = 0.0;
			    odom_msg.twist.twist.linear.z = 0.0;
			    odom_msg.twist.twist.angular.x = 0.0;
			    odom_msg.twist.twist.angular.y = 0.0;
			    odom_msg.twist.twist.angular.z = _actual_wz;
    			_odom_pub->publish(odom_msg);

			    // Stuff and publish /tf
				auto odom_tf_msg = geometry_msgs::msg::TransformStamped();
				odom_tf_msg.header.frame_id = odom_msg.header.frame_id;
				odom_tf_msg.child_frame_id = odom_msg.child_frame_id;			    
			    odom_tf_msg.header.stamp = odom_msg.header.stamp;
			    odom_tf_msg.transform.translation.x = _x;
			    odom_tf_msg.transform.translation.y = _y;
			    odom_tf_msg.transform.translation.z = 0.0;
			    odom_tf_msg.transform.rotation.x = q.x();
			    odom_tf_msg.transform.rotation.y = q.y();
			    odom_tf_msg.transform.rotation.z = q.z();
			    odom_tf_msg.transform.rotation.w = q.w();
			    _br.sendTransform(odom_tf_msg);

		        // reply
		        write();
		    }
		    else
		    {
		        RCLCPP_INFO(this->get_logger(), "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
		    }
		    msg_len = 0;
		}
		prev_byte = byte;
	}

	void write()
	{
	    if(_port_fd == -1)
	    {
        	RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        	return;
    	}

    	// filter speed feedback
    	static float const alpha = 0.5f;
		_actual_vx_filtered = (1.0f-alpha)*_actual_vx_filtered+alpha*_actual_vx;
		_actual_wz_filtered = (1.0f-alpha)*_actual_wz_filtered+alpha*_actual_wz;

    	// speed profil
    	static float const dt = 0.010; // 10ms UART feedback rate
		if(_setpoint_vx>_setpoint_vx_profil)
			_setpoint_vx_profil = std::min(_setpoint_vx_profil+_acceleration*dt,_setpoint_vx);
		else
			_setpoint_vx_profil = std::max(_setpoint_vx_profil-_deceleration*dt,_setpoint_vx);

    	// control
    	float x_error = _setpoint_vx_profil - _actual_vx_filtered;
    	float w_error = _setpoint_wz - _actual_wz_filtered;

    	float x_speed = _x_pid.process(x_error,_setpoint_vx_profil);
    	float w_speed = _w_pid.process(w_error,_setpoint_wz);

    	// trace
    	geometry_msgs::msg::Twist m;
    	m.linear.x = x_speed;
    	m.angular.z = w_speed;
		_speed_setpoint_pub->publish(m);

    	// prepare command
	    serial_command command;
	    command.start = (uint16_t)START_FRAME;
	    command.left_speed = std::clamp( (int16_t)(-x_speed-w_speed), (int16_t)-1000, (int16_t)1000);
	    command.right_speed = std::clamp( (int16_t)(-x_speed+w_speed), (int16_t)-1000, (int16_t)1000);
	    command.checksum = (uint16_t)(command.start ^ command.left_speed ^ command.right_speed);

    	int rc = ::write(_port_fd, (const void*)&command, sizeof(serial_command));
    	if (rc < 0)
    	{
	        RCLCPP_ERROR(this->get_logger(), "Error writing to hoverboard serial port");
	    }
}

private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _speed_setpoint_pub;
	tf2_ros::TransformBroadcaster _br;

	void _cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) //, const std::string & key)
	{
		//RCLCPP_INFO(this->get_logger(),"CMD_VEL %.3f %.3f",msg->linear.x,msg->angular.z);
		_setpoint_vx = msg->linear.x;
		_setpoint_wz = msg->angular.z;
	}

	// setpoints
	float _setpoint_vx = 0.0f;
	float _setpoint_wz = 0.0f;

	// setpoint with acc/speed profil
	float _acceleration = 1.0f;
	float _deceleration = 0.5f;
	float _setpoint_vx_profil = 0.0f;

	// actual
	float _actual_vx = 0.0f;
	float _actual_wz = 0.0f;

	// actual filtered
	float _actual_vx_filtered = 0.0f;
	float _actual_wz_filtered = 0.0f;

	// odometry
	float _x = 0.0f;
	float _y = 0.0f;
	float _heading = 0.0f;

	// closed-loop control
	float _x_kp = 0.0f;
	float _w_kp = 0.0f;
	float _x_ki = 0.0f;
	float _w_ki = 0.0f;
	float _x_kd = 0.0f;
	float _w_kd = 0.0f;
	float _x_kff = 100.0f;
	float _w_kff = 100.0f;
	float _x_d_alpha = 1.0f;
	float _w_d_alpha = 1.0f;
	float _x_o_alpha = 1.0f;
	float _w_o_alpha = 1.0f;
	pid_ff_controller _x_pid;
	pid_ff_controller _w_pid;

    // hoverboard protocol variables
    int _port_fd = 0;	
    uint16_t start_frame = 0;
    size_t msg_len = 0;
    uint8_t prev_byte = 0; 
    uint8_t * p = nullptr;
    serial_feedback msg;
    serial_feedback prev_msg;

};

int main(int argc, char ** argv)
{
	rclcpp::init(argc,argv);
	auto hoverboard_node = std::make_shared<driver_node>();
	////rclcpp::spin(hoverboard_node);
	while (rclcpp::ok())
	{
		hoverboard_node->read();
		//hoverboard_node->write();
		rclcpp::spin_some(hoverboard_node);
	}  	
	rclcpp::shutdown();
	return 0;
}