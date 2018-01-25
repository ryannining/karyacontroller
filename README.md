#karyacontroller
I make this project to learn about the controller.
##Description
simple multi axis motor controller for use in cnc project.
This project is possible because other people work:
- Teacup (gcode parsing, sersendp_f)
- notgrbl (simple serial reading)
- google

its not 100% exact code, but i have read lots of their code, so my code will effected by them

###Feature

feature i need to have in this software is: (* implemented)
- Per motor step using float speed ramping acceleration (smooth ) *
- per axis acceleration *
- per axis max feedrate *
- per axis jerk  *
- path planner (forward) *
- step per mm for each axis *
- implement motor stepper hardware layer *
- implement GCODE parser and processing *
- implement flow control between motion gcode and non motion gcode *
- 4 axis X Y Z E *
- implement endstop reading (xmin xmax , etc)*
- eeprom configuration (step/mm, accel, travel accel, jerk, max axis) *
- heater with PID E0 *
- Async temp reading *
- config files to set pins and parameter *
- G0 and G1 can have different acceleration (travel vs feed/extrude) *
- backlash for all motor *
- Config for inverted motor and endstop

- backward planner
- interrupt timing

## MCU
- Nano V3 328p * tested
- Mega 2560 *
- Wemos D1 *
- STM32 bluepill (WIP)

## EEPROM
Yes, its support EEPROM and modification in repetier host

## WIFI
Yes, work in progress, especially for ESP8266

## Drive system

* implemented
- Cartesian* 
- Corexy*
- Corexz*

-Delta
-Scara

For delta and scara, i still confuse how to implement segment slicing of the path. If i do the segmentation before the path planner, then a
simple straight line will quickly fill the move buffer and prevent for good planning and eat a lot resource (especially on low CPU).

I wish i can implement the segmentation after the planner. So its already know where and when to ramp up/down, then just slice it as delta movement.

for now it still not use timer interrupt, just use micros() to control the timing..



Project files:
karyacontroller.project - just for editing in codelite
karyacontroller.ino - arduino ide
motion [motion.exe].PRJ - for quincy IDE, we can show graphics of the motion simulation here

Original code is in freebasic, you can check the old folder