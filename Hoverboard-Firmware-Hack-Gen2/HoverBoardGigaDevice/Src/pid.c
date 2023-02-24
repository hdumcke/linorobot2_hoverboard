#include "../Inc/pid.h"

float last_error = 0;
float Kp = -1; // Proportial
float Ki = -1; // Integral
float Kd = -1; // Differential
float Iterm = 0; // Remember Integral of error
const float windup_guard = 1000;

int16_t updatePID(const float desired_val, const float measured_val, const float dt)
{
	float error;
	float delta_error;
	float Dterm;
	
	if (Kp == -1 || Ki == -1 || Kp == -1)
	{
		// PID values not initialized
		return 0;
	}
	error = desired_val - measured_val;
	delta_error = error - last_error;
	Iterm += error * dt;
	
	// Prevent I from getting too large
	if (Iterm < - windup_guard)
	{
		Iterm = - windup_guard;
	}
	else if (Iterm > windup_guard)
	{
		Iterm = windup_guard;
	}
	Dterm = 0;
	if (dt > 0)
	{
		Dterm = delta_error / dt;
	}
	// Remember stuff for next calculation
	last_error = error;
	// Compute actual PWM output
	return (int16_t) (Kp*error + Ki*Iterm + Kd*Dterm);
}