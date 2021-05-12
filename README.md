# Programmer for 8-bit PIC devices built upon STM8S003F3P6, based on the original AVR version

## Hardware

STM8S003F3P6, any USB to UART connected to UART1 PD5/PD6 (TX/RX).

| STM8S003F3P6 pin | Target PIC pin | Comment                                                                     |
|------------------|----------------|-----------------------------------------------------------------------------|
| VSS              | GND            | common ground                                                               |
| VDD              | VDD            | optional*- power supply for PIC MCU, you may power target from other source |
| PC5              | MCLR           | reset line of PIC MCU                                                       |
| PC4              | PGD            | programming data line                                                       |
| PC3              | PGC            | programming clock line                                                      |

## Software

See orginal AVR version https://github.com/jaromir-sukuba/a-p-prog

### Added experimental options

-r : read flash, eeprom (only for 12f1822 for now, 'Read Data From Data Memory' 05h programming command), config. Example:

`$ ./pp3 -c /dev/ttyACM0 -t 12f1822 -r`

-b n : set baud rate. Default 115200.


## Supported devices

See orginal AVR version https://github.com/jaromir-sukuba/a-p-prog

Tested by myself:

**DEVICE**|**TESTED**|**NOTES**
:-----:|:-----:|:-----:
PIC12F1822|YES|
PIC16F18326|YES|Programming only

The whole project is licensed under MIT license, see LICENSE.md file.
Some more details to be found here https://hackaday.io/project/8559-pic16f1xxx-arduino-based-programmer
