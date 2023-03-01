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

#include "gd32f1x0.h"
#include "../Inc/it.h"
#include "../Inc/comms.h"
#include "../Inc/commsSteering.h"
#include "../Inc/setup.h"
#include "../Inc/config.h"
#include "../Inc/defines.h"
#include "../Inc/bldc.h"
#include "../Inc/led.h"
#include "stdio.h"
#include "string.h"

// Only master communicates with steerin device
#ifdef MASTER
#define USART_STEER_TX_BYTES 15   // Transmit byte count including start '/' and stop character '\n'
#define USART_STEER_RX_BYTES 11   // Receive byte count including start '/' and stop character '\n'

extern uint8_t usartSteer_COM_rx_buf[USART_STEER_COM_RX_BUFFERSIZE];
static uint8_t sSteerRecord = 0;
static uint8_t sUSARTSteerRecordBuffer[USART_STEER_RX_BYTES];
static uint8_t sUSARTSteerRecordBufferCounter = 0;
int16_t led_slave;
int16_t back_led_slave;
void CheckUSARTSteerInput(uint8_t u8USARTBuffer[]);
// Config Params
extern float Ki;
extern float Kd;
extern float Kp;
extern int32_t speedM; // speed master
extern int32_t speedS; // speed slave

extern int32_t encM; // speed master
extern int32_t encS; // speed slave
uint8_t sendSteerIdentifier = 0;
extern float realSpeed;
// Battery voltage and dc
extern float batteryVoltage; 							// global variable for battery voltage
extern float currentDC; 									// global variable for current dc
extern int16_t currentDCSlave;
extern int16_t debugSlave;
extern int16_t debugMaster;

extern uint8_t buzzerFreq;    						// global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; 						// global variable for the buzzer pattern. can be 1, 2, 3, 4, 5
//----------------------------------------------------------------------------
// Send frame to steer device
//----------------------------------------------------------------------------
void SendSteerDevice(void)
{
	uint16_t crc;
	int16_t bat_voltage;
	int16_t bat_curr;
	int index = 0;
	uint8_t buffer[USART_STEER_TX_BYTES];
	int16_t value;
	// Decide which process value has to be sent
	switch(sendSteerIdentifier)
	{
		case BAT_U:
			value = batteryVoltage * 100;
			break;
		case MOT_M_I:
			value = currentDC * 100;
			break;
		case MOT_S_I:
			value = currentDCSlave;
			break;
		case MOT_M_D:
			value = (int16_t)(debugMaster);
			break;
		case MOT_S_D:
			value = (int16_t)(debugSlave);
			break;	
		default:
				break;
	}

	// Ask for steer input
	buffer[index++] = '/';
	// encM
	buffer[index++] = (encM >> 24) & 0xFF;
	buffer[index++] = (encM >> 16) & 0xFF;
	buffer[index++] = (encM >> 8) & 0xFF;
	buffer[index++] =  encM & 0xFF;
	// encS
	buffer[index++] = (encS >> 24) & 0xFF;
	buffer[index++] = (encS >> 16) & 0xFF;
	buffer[index++] = (encS >> 8) & 0xFF;
	buffer[index++] =  encS & 0xFF;
	buffer[index++] = sendSteerIdentifier;
	buffer[index++] = (value >> 8) & 0xFF;
	buffer[index++] = value & 0xFF;

	// Calculate CRC (first bytes except crc and stop byte)
	crc = CalcCRC(buffer, index);
	buffer[index++] = (crc >> 8) & 0xFF;
	buffer[index++] =  crc & 0xFF;
	buffer[index++] = '\n';
	SendBuffer(USART_STEER_COM, buffer, index);
	// Increment identifier
	sendSteerIdentifier++;
	if (sendSteerIdentifier > 4)
	{
		sendSteerIdentifier = 0;
	}
}

//----------------------------------------------------------------------------
// Update USART steer input
//----------------------------------------------------------------------------
void UpdateUSARTSteerInput(void)
{
	uint8_t character = usartSteer_COM_rx_buf[0];
	
	// Start character is captured, start record
	if (character == '/')
	{
		sUSARTSteerRecordBufferCounter = 0;
		sSteerRecord = 1;
	}

	if (sSteerRecord)
	{
		sUSARTSteerRecordBuffer[sUSARTSteerRecordBufferCounter] = character;
		sUSARTSteerRecordBufferCounter++;
		
		if (sUSARTSteerRecordBufferCounter >= USART_STEER_RX_BYTES)
		{
			sUSARTSteerRecordBufferCounter = 0;
			sSteerRecord = 0;
			
			// Check input
			CheckUSARTSteerInput (sUSARTSteerRecordBuffer);
		}
	}
}

//----------------------------------------------------------------------------
// Check USART steer input
//----------------------------------------------------------------------------
void CheckUSARTSteerInput(uint8_t USARTBuffer[])
{
	// Auxiliary variables
	uint16_t crc;
	uint8_t identifier;
	int16_t config_value;
	
	// Check start and stop character
	if ( USARTBuffer[0] != '/' ||
		USARTBuffer[USART_STEER_RX_BYTES - 1] != '\n')
	{
		return;
	}
	
	// Calculate CRC (first bytes except crc and stop byte)
	crc = CalcCRC(USARTBuffer, USART_STEER_RX_BYTES - 3);
	
	// Check CRC
	if ( USARTBuffer[USART_STEER_RX_BYTES - 3] != ((crc >> 8) & 0xFF) ||
		USARTBuffer[USART_STEER_RX_BYTES - 2] != (crc & 0xFF))
	{
		return;
	}
	
	// Calculate result joint speed value -2000 to 2000 mm/s
	speedM = CLAMP((int16_t)((USARTBuffer[1] << 8) | USARTBuffer[2]), -2000, 2000);
	// Calculate result joint speed value -2000 to 2000 mm/s
	speedS = CLAMP((int16_t)((USARTBuffer[3] << 8) | USARTBuffer[4]), -2000, 2000);
	identifier = USARTBuffer[5];
	config_value = (int16_t)((USARTBuffer[6] << 8) | USARTBuffer[7]);
	CheckConfigValue(identifier, config_value);
	// Reset the pwm timout to avoid stopping motors
	ResetTimeout();
}


//----------------------------------------------------------------------------
// Checks input value from master to set value depending on identifier
//----------------------------------------------------------------------------
void CheckConfigValue(uint8_t identifier, int16_t value)
{
	switch(identifier)
	{
		case PID_P:
			Kp = (float)value / 100;
			break;
		case PID_I:
			Ki = (float)value / 100;
			break;
		case PID_D:
			Kd = (float)value / 100;
			break;
		case LED_M:
			SetRGBProgram(value);
			break;
		case BACK_LED_M:
			// 0 = Off, 1 = Green, 2 = Red, 3 = Orange
			switch(value)
			{
				case 1:
					EnableLEDPin(LED_GREEN);
					break;
				case 2:
					EnableLEDPin(LED_RED);
					break;
				case 3:
					EnableLEDPin(LED_ORANGE);
					break;
				default:
					// Turn off LED
					EnableLEDPin(0);
			}
			break;
		case LED_S:
			// Set global variable for slave
			led_slave = value;
			break;
		case BACK_LED_S:
			// Set global variable for slave
			back_led_slave = value;
			break;
		case BUZZER:
			buzzerFreq = value;
      buzzerPattern = 1;
			break;
		default:
			break;
	}
}


#endif
