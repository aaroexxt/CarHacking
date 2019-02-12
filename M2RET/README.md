M2RET
=======

Reverse Engineering Tool running on Macchina M2 Hardware

A fork of the GVRET project.

#### Requirements:

You will need the following to be able to compile the run this project:

- [Arduino IDE](https://www.arduino.cc/en/Main/Software) 1.6.0 or higher (tested all of the way up to 1.8.5)
- [M2 Board Configuration](https://github.com/macchina/arduino-boards-sam) - Needed to define the pins for various hardware
- [due_can](https://github.com/collin80/due_can) - Object oriented canbus library for Arduino Due compatible boards.
- [due_wire](https://github.com/collin80/due_wire) - An alternative I2C library for Due with DMA support.
- [M2_SD_HSMCI](https://github.com/macchina/M2_SD_HSMCI) - SD card support library
- [LIN](https://github.com/macchina/LIN) - Support for the dual LIN ports on the M2
- [Single-Wire CAN (MCP2515)](https://github.com/macchina/Single-Wire-CAN-mcp2515) - Single wire CAN support for M2 board
- [MCP2515](https://github.com/collin80/mcp2515) - CAN library that powers the Single-Wire CAN library
- [can_common](https://github.com/collin80/can_common) - Base CAN library used by due_can and mcp2515
- [M2_12VIO](https://github.com/TDoust/M2_12VIO) - Macchina M2 12VIO library

All libraries belong in %USERPROFILE%\Documents\Arduino\libraries (Windows) or ~/Arduino/libraries (Linux/Mac).
You will need to remove -master or any other postfixes. Your library folders should be named as above.

The canbus is supposed to be terminated on both ends of the bus. This should not be a problem as this firmware will be used to reverse engineer existing buses. However, do note that CAN buses should have a resistance from CAN_H to CAN_L of 60 ohms. This is affected by placing a 120 ohm resistor on both sides of the bus. If the bus resistance is not fairly close to 60 ohms then you may run into trouble.  

#### The firmware is a work in progress. What works:
- CAN0 and CAN1 are operational
- Single wire CAN
- EEPROM can be used to save settings between start ups
- Text console is active (configuration and CAN capture display)
- Can connect as a GVRET device with SavvyCAN
- Able to automatically start up and log all traffic to sdCard. Not stable at high bus loads just yet.
- LAWICEL support (somewhat tested. Still experimental)
- Blinken Lights!
- Added ADC DMA support. digital I/O pins on the 26 pin connector
- Added optional support for Macchina M2_12VIO library


#### What does not work:
- Either LIN bus
- Anything you attach to the XBEE port

#### License:

This software is MIT licensed:

Copyright (c) 2014-2017 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

