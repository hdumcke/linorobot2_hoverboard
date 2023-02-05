// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "horo_hardware/hoverboard_proxy.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/types.h>

#include "rclcpp/rclcpp.hpp"


namespace horo_hardware
{
hardware_interface::CallbackReturn ProxyClient::on_activate()
{
	int data_len = 0;
	struct sockaddr_un remote;
        static const char* socket_path = SOCKET_NAME;

	if( (sock = socket(AF_UNIX, SOCK_SEQPACKET, 0)) == -1  )
	{
            RCLCPP_FATAL(
              rclcpp::get_logger("DiffBotSystemHardware"),
              "Client: Error on socket() call %s\n", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
	}

	remote.sun_family = AF_UNIX;
	strcpy( remote.sun_path, socket_path );
	data_len = strlen(remote.sun_path) + sizeof(remote.sun_family);

	if( connect(sock, (struct sockaddr*)&remote, data_len) == -1 )
	{
            RCLCPP_FATAL(
              rclcpp::get_logger("DiffBotSystemHardware"),
              "Client: Error on connect call %s\n", strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
	}

	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ProxyClient::on_deactivate()
{
	close(sock);
	return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ProxyClient::read()
{
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ProxyClient::write(double leftSpeed, double rightSpeed)
{
    int data_len = 0;
    char recv_msg[s_recv_len];
    char send_msg[s_send_len];

    RCLCPP_INFO(
      rclcpp::get_logger("DiffBotSystemHardware"),
      "leftSpeed %.5f rightSpeed %.5f\n", leftSpeed, rightSpeed);

    memset(recv_msg, 0, s_recv_len*sizeof(char));
    memset(send_msg, 0, s_send_len*sizeof(char));

    //TODO map speed to range of -1000 - 1000
    //wheel_radius: 0.0825 is defined in parameters
    double wheel_radius = 0.0825;
    short speedMaster = rightSpeed/wheel_radius * 100;
    short speedSlave = leftSpeed/wheel_radius * 100;
	    
    send_msg[0] = 6;
    send_msg[1] = INST_SETSPEED;
    memcpy(&send_msg[2], &speedMaster, sizeof(speedMaster));
    memcpy(&send_msg[4], &speedSlave, sizeof(speedSlave));

    if( send(sock, send_msg, send_msg[0], 0 ) == -1 )
    {
        RCLCPP_FATAL(
          rclcpp::get_logger("DiffBotSystemHardware"),
          "Client: Error on send() call \n");
        return hardware_interface::return_type::ERROR;
    }

    if( (data_len = recv(sock, recv_msg, s_recv_len, 0)) > 0 )
    {
    	return hardware_interface::return_type::OK;
    }
    else
    {
    	if(data_len < 0)
    	{
            RCLCPP_FATAL(
              rclcpp::get_logger("DiffBotSystemHardware"),
              "Client: Error on recv() call %s\n", strerror(errno));
    	}
    	else
    	{
            RCLCPP_FATAL(
              rclcpp::get_logger("DiffBotSystemHardware"),
              "Client: Server socket closed \n");
    	}

    }
    return hardware_interface::return_type::ERROR;
}

}  // namespace horo_hardware
