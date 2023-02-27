#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

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
	//_br(this),
	_x_pid(&_x_kp,&_x_ki,&_x_kd,&_x_kff,-1000.0,1000.0),
	_w_pid(&_w_kp,&_w_ki,&_w_kd,&_w_kff,-1000.0,1000.0)
	
	{
		this->declare_parameter("acc_x",_acceleration_x);
		this->declare_parameter("acc_z",_acceleration_z);

		this->declare_parameter("pwm_max",_pwm_max);

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

        _acceleration_x = this->get_parameter("acc_x").as_double();
        _acceleration_z = this->get_parameter("acc_z").as_double();

        _pwm_max = this->get_parameter("pwm_max").as_int();

        _x_kp = this->get_parameter("x_kp").as_double();
        _x_ki = this->get_parameter("x_ki").as_double();
        _x_kd = this->get_parameter("x_kd").as_double();
        _x_kff = this->get_parameter("x_kff").as_double();
        _x_d_alpha = this->get_parameter("x_d_alpha").as_double();
        _x_o_alpha = this->get_parameter("x_o_alpha").as_double();

        _w_kp = this->get_parameter("w_kp").as_double();
        _w_ki = this->get_parameter("w_ki").as_double();
        _w_kd = this->get_parameter("w_kd").as_double();
        _w_kff = this->get_parameter("w_kff").as_double();
        _w_d_alpha = this->get_parameter("w_d_alpha").as_double();
        _w_o_alpha = this->get_parameter("w_o_alpha").as_double();

        _x_pid._d_alpha = _x_d_alpha;
        _x_pid._out_alpha = _x_o_alpha;

        _w_pid._d_alpha = _w_d_alpha;
        _w_pid._out_alpha = _w_o_alpha;

		_param_cb_ptr = this->add_on_set_parameters_callback(std::bind(&driver_node::dynamic_parameters_cb, this, std::placeholders::_1));

		RCLCPP_INFO(this->get_logger(),"Starting...");
		_cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
			"/cmd_vel",
			10,
			std::bind(&driver_node::_cmd_vel_callback, this, std::placeholders::_1)
		);	

		// odometry publisher
 		_odom_pub = create_publisher<nav_msgs::msg::Odometry>(
 			"/odom_wheel",
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

		// get time
		rcutils_time_point_value_t now;
		if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
		}
		_stat_start_time_s = RCL_NS_TO_S(now);	

		_timer = this->create_wall_timer(
			std::chrono::milliseconds(5),
			std::bind(&driver_node::read, this)
		);
	}

	~driver_node()
	{
		if (_port_fd != -1) 
			close(_port_fd);
	}
    
	rcl_interfaces::msg::SetParametersResult dynamic_parameters_cb(
        const std::vector<rclcpp::Parameter> & parameters
	)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "x_kp" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_kp = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "x_ki" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_ki = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "x_kd" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_kd = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "x_kff" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_kff = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_kp" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_kp = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_ki" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_ki = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_kd" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_kd = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_kff" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_kff = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
        }
        return result;
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

				// reply
				write();

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
				odom_msg.child_frame_id = "base_footprint";
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
			    // Pose covariance (required by robot_pose_ekf)
				/* pose covariance
				    x  y  z  R  P  Y
				 x  0  1  2  3  4  5  
				 y  6  7  8  9 10 11
				 z 12 13 14 15 16 17
				 R 18 19 20 21 22 23
				 P 24 25 26 27 28 29
				 Y 30 31 32 33 34 35  
				*/
				// Pose covariance unknown = -1
				odom_msg.pose.covariance[0] = -1.0;
				// Note : we dont need a covariance matrix because EKF is configured not to use pose (x,y,z,roll,pitch,yaw) from wheel odometry

			    odom_msg.twist.twist.linear.x = _actual_vx;
			    odom_msg.twist.twist.linear.y = 0.0;
			    odom_msg.twist.twist.linear.z = 0.0;
			    odom_msg.twist.twist.angular.x = 0.0;
			    odom_msg.twist.twist.angular.y = 0.0;
			    odom_msg.twist.twist.angular.z = _actual_wz;
			    for (unsigned int i = 0; i < odom_msg.twist.covariance.size(); ++i)
			    {
      				odom_msg.twist.covariance[i] = 0.0;
    			}
			    // Twist covariance (required by robot_pose_ekf)
				/* twist covariance
				    vx vy vz wx wy wz
				 vx  0  1  2  3  4  5  
				 vy  6  7  8  9 10 11
				 vz 12 13 14 15 16 17
				 wx 18 19 20 21 22 23
				 wy 24 25 26 27 28 29
				 wz 30 31 32 33 34 35  
				*/
				odom_msg.twist.covariance[0] = 0.1; // vx variance = 0.1m/s 
				odom_msg.twist.covariance[35] = 0.05; // wz variance = 0.05rad/s ~3deg/s (must be higher thant IMU so EKF uses IMU)
				_odom_pub->publish(odom_msg);

				// stats
				++_stat_feedback_count;
				static int32_t counter = 0;
				if(++counter%1000==0)
				{
					int32_t ellapsed_time = RCL_NS_TO_S(now) - _stat_start_time_s;
					float packet_error_rate = (float)_stat_feedback_checksum_error/(float)_stat_feedback_count;
					float frequency = (float)_stat_feedback_count/(float)ellapsed_time;
					RCLCPP_INFO(this->get_logger(), "PER:%0.3f Freq:%.1f",packet_error_rate,frequency);
				}
		    }
		    else
		    {
		        //RCLCPP_INFO(this->get_logger(), "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
				// stats
				++_stat_feedback_checksum_error;
				++_stat_feedback_count;
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

		// timeout setpoints
		rcutils_time_point_value_t now;
		if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
		}		
		if( RCL_NS_TO_S(now) > (_setpoint_timestamp_s+2) ) // 2 sec time-out
		{
			_setpoint_vx = 0.0f;
			_setpoint_wz = 0.0f;
		}

		// filter speed feedback
		static float const alpha = 0.5f;
		_actual_vx_filtered = (1.0f-alpha)*_actual_vx_filtered+alpha*_actual_vx;
		_actual_wz_filtered = (1.0f-alpha)*_actual_wz_filtered+alpha*_actual_wz;

		// speed profil
		static float const dt = 0.010; // 10ms UART feedback rate
		if(_setpoint_vx>_setpoint_vx_profil)
			_setpoint_vx_profil = std::min(_setpoint_vx_profil+_acceleration_x*dt,_setpoint_vx);
		else
			_setpoint_vx_profil = std::max(_setpoint_vx_profil-_acceleration_x*dt,_setpoint_vx);
		if(_setpoint_wz>_setpoint_wz_profil)
			_setpoint_wz_profil = std::min(_setpoint_wz_profil+_acceleration_z*dt,_setpoint_wz);
		else
			_setpoint_wz_profil = std::max(_setpoint_wz_profil-_acceleration_z*dt,_setpoint_wz);

		// control
		float x_error = _setpoint_vx_profil - _actual_vx_filtered;
		float w_error = _setpoint_wz_profil - _actual_wz_filtered;

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
		command.left_speed = std::clamp( (int16_t)(-x_speed-w_speed), (int16_t)-_pwm_max, (int16_t)_pwm_max);
		command.right_speed = std::clamp( (int16_t)(-x_speed+w_speed), (int16_t)-_pwm_max, (int16_t)_pwm_max);
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
	OnSetParametersCallbackHandle::SharedPtr _param_cb_ptr;

	rclcpp::TimerBase::SharedPtr _timer;

	void _cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) //, const std::string & key)
	{
		//RCLCPP_INFO(this->get_logger(),"CMD_VEL %.3f %.3f",msg->linear.x,msg->angular.z);
		_setpoint_vx = msg->linear.x;
		_setpoint_wz = msg->angular.z;
		// get time
		rcutils_time_point_value_t now;
		if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
		}
		_setpoint_timestamp_s = RCL_NS_TO_S(now);
	}

	// setpoints
	float _setpoint_vx = 0.0f;
	float _setpoint_wz = 0.0f;
	int32_t _setpoint_timestamp_s = 0;

	// setpoint with acc/speed profil
	float _acceleration_x = 1.0f;
	float _acceleration_z = 1.0f;
	float _setpoint_vx_profil = 0.0f;
	float _setpoint_wz_profil = 0.0f;

	// actual
	float _actual_vx = 0.0f;
	float _actual_wz = 0.0f;

	// actual filtered
	float _actual_vx_filtered = 0.0f;
	float _actual_wz_filtered = 0.0f;

	// max pwm
	int32_t _pwm_max = 250;

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

	// statistics
	uint32_t _stat_feedback_count = 0;
	uint32_t _stat_feedback_checksum_error = 0;
	uint32_t _stat_start_time_s = 0;

};

int main(int argc, char ** argv)
{
	rclcpp::init(argc,argv);
	auto hoverboard_node = std::make_shared<driver_node>();
	rclcpp::spin(hoverboard_node);
	/*while (rclcpp::ok())
	{
		hoverboard_node->read();
		//hoverboard_node->write();
		rclcpp::spin_some(hoverboard_node);
		//rclcpp::spin_once(hoverboard_node);
	}  	*/
	rclcpp::shutdown();
	return 0;
}
