
#ifndef PID_H
#define PID_H

#include "config.h"
#include "defines.h"

int16_t updatePID(const float desired_val, const float measured_val, const float dt);


#endif