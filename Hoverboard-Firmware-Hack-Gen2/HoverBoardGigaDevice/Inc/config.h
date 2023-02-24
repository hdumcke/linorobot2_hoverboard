/*
* This file is part of the hoverboard-firmware-hack-V2 project. The 
* firmware is used to hack the generation 2 board of the hoverboard.
* These new hoverboards have no mainboard anymore. They consist of 
* two Sensorboards which have their own BLDC-Bridge per Motor and an
* ARM Cortex-M3 processor GD32F130C8.
*
* Copyright (C) 2018 Florian Staeblein
* Copyright (C) 2018 Jakob Broemauer
* Copyright (C) 2018 Kai Liebich
* Copyright (C) 2018 Christoph Lehnert
*
* The program is based on the hoverboard project by Niklas Fauth. The 
* structure was tried to be as similar as possible, so that everyone 
* could find a better way through the code.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CONFIG_H
#define CONFIG_H

#include "gd32f1x0.h"

// ################################################################################

//#define MASTER										  	// Select if firmware is for master or slave board
#define SLAVE 												// Select if firmware is for master or slave board
//#define HALL_INTERRUPTS								// Enable Hall interrupt reading

// ################################################################################

#define PWM_FREQ         		16000     // PWM frequency in Hz
#define PID_HZ           		100       // PID frequency in Hz

#define DEAD_TIME        		60        // PWM deadtime (60 = 1�s, measured by oscilloscope)

#define DC_CUR_LIMIT     		15        // Motor DC current limit in amps

// ################################################################################

#define DELAY_IN_MAIN_LOOP 	5         // Delay in ms

#define TIMEOUT_MS          2000      // Time in milliseconds without steering commands before pwm emergency off

#ifdef MASTER
#define INACTIVITY_TIMEOUT 	8        	// Minutes of not driving until poweroff (not very precise)

// ################################################################################

//#define BAT_LOW_LVL1     35.0       // Gently beeps, show green battery symbol above this Level.
//#define BAT_LOW_LVL2     33.0       // Battery almost empty, show orange battery symbol above this Level. Charge now! 
//#define BAT_LOW_DEAD     31.0       // Undervoltage lockout, show red battery symbol above this Level.

// cool&fun 25.2 V battery, 7 cells
#define BAT_LOW_LVL1     24.5       // Gently beeps, show green battery symbol above this Level.
#define BAT_LOW_LVL2     23.0       // Battery almost empty, show orange battery symbol above this Level. Charge now!
#define BAT_LOW_DEAD     21.7       // Undervoltage lockout, show red battery symbol above this Level.

// ################################################################################
#endif

// ###### ARMCHAIR ######
#define FILTER_SHIFT 12 						// Low-pass filter for pwm, rank k=12

#ifdef MASTER
#define SPEED_COEFFICIENT   -1
#define STEER_COEFFICIENT   1
#endif

#endif
