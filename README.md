# Programmer for 8-bit PIC devices built upon STM8S003F3P6, based on original AVR version

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

When running under Linux, download source from this repository and run

`gcc -Wall pp3.c -o pp3`

This should build executable pp3. When working under windows, you can download compiled binary from this repository. Alternatively, you can build it from source - install minGW and run

`gcc -Wall pp3.c -o pp3`

ie. the same procedure as on Linux. This should result in silent build with pp3.exe executable created.

Running the executable with no parameters should only bring banner "pp programmer". Though running with basic set of parameters

`./pp3 -c /dev/ttyACM0 -t 16f1829 file.hex`

under Linux, where -c parameter denotes port to be accessed, -t parameter defines PIC to be programmed and last parameter is hex file to be downloaded; or

`pp3.exe -c COM30 -t 16f1829 file.hex`

under Windows should program the target PIC; with expected result:

	$ ./pp3 -c /dev/ttyACM0 -t 16f1829 file.hex
    Opening serial port
    Device ID 0x27E4
    Programming FLASH (16384 B in 256 pages)
    Programming config
    Verifying FLASH (16384 B in 256 pages)
    Verifying config
    Releasing MCLR


## Notes on software

You may omit the actual programming using -p switch or verification using -n switch, when using both the programmer only checks target device signature and exits.

`$ ./pp3 -c /dev/ttyACM0 -p -n -t 16f1829 file.bin`

    Opening serial port
    Device ID 0x27E4
    Releasing MCLR

you can add some debug output info using -v parameter, ranging from -v 1 to -v 4. It may be suitable for debugging, -v 4 prints out all byte transaction on serial port, so be prepared for huge output.
There is database file pp3_devices.dat which hold information of supported PIC types. For now, the filename is fixed in code can't be changed and file has to be in the same directory as pp executable.

### Experimental options

-r : read flash, eeprom (only for 12f1822 for now, 'Read Data From Data Memory' 05h programming command), config:

`$ ./pp3 -c /dev/ttyACM0 -t 12f1822 -r`

-b n : set baud rate. Default 115200.


## Supported devices

Obviously, there is more supported than verified (tested) devices. I tried to test at least one device from specific family. The members from one family are usually very very similar, giving me enough confidence to mark them as supproted. Of course, typos can happen, but those are easy to correct.

**DEVICE**|**TESTED**|**NOTES**
:-----:|:-----:|:-----:
PIC12F1822|YES|
PIC16F18326|YES|Programming only

All other MCUs may working accordingly to the original AVR version.

The whole project is licensed under MIT license, see LICENSE.md file.
Some more details to be found here https://hackaday.io/project/8559-pic16f1xxx-arduino-based-programmer
