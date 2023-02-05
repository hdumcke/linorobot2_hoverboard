#ifndef HORO_HARDWARE__HOVERBOARD_PROXY_HPP_
#define HORO_HARDWARE__HOVERBOARD_PROXY_HPP_

#include "hardware_interface/system_interface.hpp"

#define INST_SETSPEED 0x01
#define INST_GETBATT 0x02
#define INST_GETCURR 0x03
#define INST_GETSPEED 0x04

#define SOCKET_NAME "/tmp/hoverboard-proxy.socket"

namespace horo_hardware
{
class ProxyClient
{
public:
    hardware_interface::CallbackReturn on_activate();
    hardware_interface::CallbackReturn on_deactivate();
    hardware_interface::return_type read();
    hardware_interface::return_type write(double leftSpeed, double rightSpeed);

private:
    int sock = 0;
    static const unsigned int s_recv_len = 20;
    static const unsigned int s_send_len = 10;
};

}  // namespace horo_hardware

#endif  // HORO_HARDWARE__HOVERBOARD_PROXY_HPP_
