#ifndef _HOVERBOARD_PROTOCOL_H
#define _HOVERBOARD_PROTOCOL_H

#define START_FRAME 0xABCD

typedef struct {
   uint16_t start;
   int16_t  left_speed;    // [-1000,+1000]
   int16_t  right_speed;   // [-1000,+1000]
   uint16_t checksum;
} serial_command;

typedef struct {
   uint16_t start;
   int16_t  cmd1;             // steer/left speed after normalizing
   int16_t  cmd2;             // right speed after normalizing
   int16_t  right_speed_meas; // RPM
   int16_t  left_speed_meas;  // RPM
   int16_t  voltage;          // V x 100
   int16_t  temperature;      // T x 10
   uint16_t cmd_led;
   uint16_t checksum;
} serial_feedback;

#endif // _HOVERBOARD_PROTOCOL_H