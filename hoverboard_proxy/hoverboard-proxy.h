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
    s16 leftSpeed;
    s16 rightSpeed;
};

// frame parameters format for control acknowledge
struct parameters_control_acknowledge_format
{
    float battery;
    float currentMaster;
    float speedMaster;
    float currentSlave;
    float speedSlave;
};

#define INST_SETSPEED 0x01
#define INST_GETBATT 0x02
#define INST_GETCURR 0x03
#define INST_GETSPEED 0x04

#endif //_esp32_proxy__H
