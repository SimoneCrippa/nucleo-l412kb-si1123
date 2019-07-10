# nucleo-l412kb-si1123
Firmware for STM Nucleo-L412KB board to access Si1123 sensor via I2C

* Plug the board to the PC via USB/ST-Link.
* Open the virtual serial device (e.g. '/dev/ttyACM0')
* The firmware will print out, every second, a line with three values: visibility, IR, UV index.
