# The DrawBot Badge



<img src="http://www.buildlog.net/blog/wp-content/uploads/2018/10/DrawBot_01.jpg" width="500">

### Overview

The Drawbot Badge is a part of a “Getting Started in Small Scale CNC” workshop for the 2018 Hackaday Superconference. The PCB controller part packs just about every feature a small scale 3 axis CNC could ever need. The drawing machine part was provided as just the cheapest and easiest to build CNC machine I could think of. It draws wiggly line drawings on Post It Notes.

The first part of the workshop cover basic concepts of small scale CNC machines and goes over how to control thing via gcode.

During the second part of the workshop everyone builds the drawing badge.

### Badge Features

- ESP32 Controller running a fork of Grbl_ESP32
- (3) TI DRV8825 Stepper motor drivers
- (3) Hobby servo connectors
- (3) Home/Limit switch connectors
- (4) Control switch inputs
- (1) Touch probe interface
- Spindle speed control output (3.3v PWM)
- Laser Module interface (safety interlock and power control)
- (1) General purpose output (3.3v TTL)
- (1) High current output control (3A contiuous)
- Bluetooth
- Wifi (Access point or Client)
- Web page interface
- Micro SD Card Socket
- Shitty add on interface
- Power via battery, USB or DC barrel jack

**Note:** Some of the above features cannot be used at the same time. For example: For each of the 3 axes (XYZ) you must choose to use either a stepper motor or hobby servo. You cannot use all 6 items at once.  There are more details in the wiki.

### Assembly instructions

[See the wiki](https://github.com/bdring/DrawBot_Badge/wiki/Assembly-Instructions)

### Status

All source code and files will be added to the repo soon.