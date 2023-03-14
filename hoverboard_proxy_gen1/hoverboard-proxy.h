/* Authors :
 * - Hdumcke
 * - Pat92fr
 */
#ifndef _esp32_proxy__H
#define _esp32_proxy__H

#define SOCKET_NAME "/tmp/hoverboard-proxy.socket"

enum {PID_P, PID_I, PID_D, LED_S, LED_M, BACK_LED_S, BACK_LED_M, BUZZER};

enum {BAT_U, MOT_S_I, MOT_M_I, MOT_S_D, MOT_M_D};

// types
typedef char s8;
typedef unsigned char u8;
typedef unsigned short u16;
typedef short s16;
typedef unsigned long u32;
typedef long s32;

#pragma pack(1)

// frame parameters format for control instruction
struct parameters_control_instruction_format
{
    s16 rightSpeed;
    s16 leftSpeed;
    s16 pid_p;
    s16 pid_i;
    s16 pid_d;
    s16 led_l;
    s16 led_r;
    s16 back_led_l;
    s16 back_led_r;
    s16 buzzer;
};

// frame parameters format for control acknowledge
struct parameters_control_acknowledge_format
{
    u16 responseId;
    s32 speedMaster;
    s32 speedSlave;
    s16 battery;
    s16 currentMaster;
    s16 debugMaster;
    s16 currentSlave;
    s16 debugSlave;
};

#define INST_SETSPEED 0x01
#define INST_SETPID 0x02
#define INST_SETLED 0x03
#define INST_SETBACK_LED 0x04
#define INST_SETBUZZER 0x05
#define INST_GETSPEED 0x06
#define INST_GETBATT 0x07
#define INST_GETCURR 0x08
#define INST_GETDEBUG 0x09
#define INST_GETCB 0x0A

#endif //_esp32_proxy__H
