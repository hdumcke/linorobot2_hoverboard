This fork of Hoverboard Firmware Generation 2 for generic hoverboards using two mainboards has the following changes compared to the original firmware:
````
- Seperate control of each motor
- Added PID controllers allowing for each motor to be controlled using a real speed instead of a power level (P, I and D gains still have to be tweaked better, power buildup at lower speeds is pretty slow)
- All important data is now send to the steering devices (battery voltage, master and slave current and master and slave realSpeed)
````

To control the hoverboard drivers, a development board, like for example an Arduino (3.3V logic levels!!!) has to be connected to the master controller using a UART port. It's recommended to have a board with multiple UART ports like for example the Arduino Due (connected directly) or Arduino Mega (using logic level converters). Check out the reversed-engineered schematics for "REMOTE" down below. The overview of all communications can be found in `CommunicationOverview.ods`. The function for generating the 16 bit CRC check can be found in `HoverBoardGigaDevice/src/comms.c`. The typedef `FLOATUNION_t` in C for converting seperate float bytes to a float variable can be found in `HoverBoardGigaDevice/inc/defines.h`. 

Data can be read out either by using the master controller or the slave controller. When using the master controller, 24 bytes of data containing the battery voltage and master and slave speeds and currents can just be read out. Each value is in a float format split up into 4 bytes. This can be converted using the `FLOATUNION_t` typedef. When using the slave controller, data first has to be requested before it is send. It is also possible to change certain settings on the hoverboard controllers like for example enabling or disabling the beeping when driving backwards. To read out data you have to set the readWrite field to '0' and to change settings you have to set it to '1'. Notice that for every data send in the slave controller you have to use ASCII character instead of real numbers. You can chose which data has to send back or which value in the hoverboard has to be changed by sending the correct identifier. The list with the correct identifiers can be found for reading in the first switch statement and for writing in the second switch statement in the source file `HoverBoardGigaDevice/src/commsBluetooth.c`. 

All communications happen using a baudrate of 19200. The main controller has to receive data at least every 2000 milliseconds before it emergency shuts off. 

__________________________________________

### Hoverboard-Firmware-Hack-Gen2

Hoverboard Hack Firmware Generation 2 for the Hoverboard with the two Mainboards instead of the Sensorboards (See Pictures).

This repo contains open source firmware for generic Hoverboards with two mainboards. It allows you to control the hardware of the new version of hoverboards (like the Mainboard, Motors and Battery) with an arduino or some other steering device for projects like driving armchairs.

The structure of the firmware is based on the firmware hack of Niklas Fauth (https://github.com/NiklasFauth/hoverboard-firmware-hack/). Because of a different model of processor (GD32F130C8 instead of STM32F103) it was not possible to use the same firmware and it has to be written from scratch (Different hardware, different, pins, different registers :( )

- This project requires knowledge of the initial project linked above.
- At the current point I am not able to support any questions or issues - sorry!

---

#### Hardware
![otter](https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2/blob/master/Hardware_Overview_small.png)

The hardware has two main boards, which are different equipped. They are connected via USART. Additionally there are some LED PCB connected at X1 and X2 which signalize the battery state and the error state. There is an programming connector for ST-Link/V2 and they break out GND, USART/I2C, 5V on a second pinhead.

The reverse-engineered schematics of the mainboards can be found here:
https://github.com/flo199213/Hoverboard-Firmware-Hack-Gen2/blob/master/Schematics/HoverBoard_CoolAndFun.pdf


---

#### Flashing
The firmware is built with Keil (free up to 32KByte). To build the firmware, open the Keil project file which is includes in repository. Right to the STM32, there is a debugging header with GND, 3V3, SWDIO and SWCLK. Connect GND, SWDIO and SWCLK to your SWD programmer, like the ST-Link found on many STM devboards.

- If you never flashed your mainboard before, the controller is locked. To unlock the flash, use STM32 ST-LINK Utility or openOCD.
- To flash the STM32, use the STM32 ST-LINK Utility as well, ST-Flash utility or Keil by itself.
- Hold the powerbutton while flashing the firmware, as the controller releases the power latch and switches itself off during flashing
