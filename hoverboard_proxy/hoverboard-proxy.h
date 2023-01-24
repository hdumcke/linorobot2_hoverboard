/* Authors :
 * - Hdumcke
 * - Pat92fr
 */
#ifndef _esp32_proxy__H
#define _esp32_proxy__H

#define SOCKET_NAME "/tmp/hoverboard-proxy.socket"

// types
typedef char s8;
typedef unsigned char u8;
typedef unsigned short u16;
typedef short s16;
typedef unsigned long u32;
typedef long s32;

// frame parameters format for control instruction
struct parameters_control_instruction_format
{
    u16 speed;
    u16 steer;
};

// frame parameters format for control acknowledge
struct parameters_control_acknowledge_format
{
    u16 dummy;
};

#define INST_SETSPEED 0x01

#endif //_esp32_proxy__H
