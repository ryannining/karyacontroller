# karyacontroller
I make this project to learn about the controller.
## Description
simple multi axis motor controller for use in cnc project.
This project is possible because other people work:
- Teacup (gcode parsing, sersendp_f)
- notgrbl (simple serial reading)
- google

its not 100% exact code, but i have read lots of their code, so my code will effected by them

### Feature

feature i need to have in this software is: (* implemented)
- per axis max feedrate *
- path planner (backward and forward) *
- step per mm for each axis *
- motor stepper hardware layer using GPIO and ShiftRegister *
- GCODE parser and processing *
- 4 axis X Y Z E *
- eeprom configuration (step/mm, accel, travel accel, jerk, max axis) *
- implement endstop reading (xmin xmax , etc)*
- implement Probing Z ()*
- implement Mesh autolevel ()*
- Endstop configuration using EEPROM *
- heater with PID E0 PWM or Bang Bang*
- temp sensor math model (heating pid without real sensor) *
- Async temp reading *
- config files contain board definition to set pins and parameter *
- G0 and G1 can have different acceleration (travel vs feed/extrude) *
- backlash compensation *
- Config for inverted motor and endstop *
- interrupt timing * (for AVR, ESP8266, STM32F103) 
- NEW: Laser constant burn per step, by only turn on the laser on specific time then turn off on each motor step *
- NEW: G7 Raster data for Laser Engraving for optimum speed laser engraving *
- NEW: M290 Baby stepping for 3d printing *
- NEW: M206 Pxxx [sss] xxx=380 400 450 470 configure wifi and telegram client, M504 to show setting*
- NEW: Retract conversion, just set the retract speed to 1 on slicer it will be converted to retract setting on eeprom. *
- NEW: M2 will stop the code immediately, without waiting buffer emptied, but still know the correct position *
- IDEA: Implement clever gcode sender that pre-compile the gcode by doing path planner and send the plan to the firmware


## MCU
- Nano V3 328p * tested
- Mega 2560 * Tested
- Wemos D1 mini * Tested
- STM32 bluepill * tested

## EEPROM
Yes, its support EEPROM and modification in repetier host. 

## WIFI
Still work in progress, especially for ESP8266
Progress:

- Websocket server at port 81, receive Ping Pong communication
- Html gcode sender tools, integrated with CNC gcode generator. (open the http://ipaddress)
- Have print and run gcode for CNC
- Still have lag, i dont know where is the problem, maybe my Access point ?


## Drive system

* implemented
LINEAR
- Cartesian* 
- Corexy*
- Corexz*
- XYYZ* (dual motor for Y that can do individual movement when homing)

NONLINEAR
-Delta*
-Deltasian*
-Scara

## Other

Subpixel bresenham algorithm, will increase the resolution of the bresenham especially on low speed movement. just like AMASS on GRBL.

Non linear slicing happen in the core of movement. By implementing a virtual step/mm and calculate real step in the motion loop. So 1 Gcode will save in 1 path buffer, then segmented when this buffer is active.

for now it already use timer interrupt, but still can use polling mode using micros() to control the timing (especially for new mcu that we dont know how to use timer, or for simulation in pc)..

## Files

Project files:
karyacontroller.project - just for editing in codelite
karyacontroller.ino - arduino ide
motion [motion.exe].PRJ - for quincy IDE, we can show graphics of the motion simulation here

Original code is in freebasic, you can check the old folder

## log

5-11-2019
Multiple job stored on flash, and more other feature.
![image](https://user-images.githubusercontent.com/11457832/68217253-e33d1b00-0014-11ea-83b4-d407353b8957.png)

5-3-2019
Auto level using MESH bilinear interpolation. 

10-12-2018
![image](https://user-images.githubusercontent.com/11457832/49775685-83b68c80-fd2b-11e8-8161-39acdba5641b.png)
Able to upload gcode to internal wemos flash, and start print without pc, without sdcard

20-5-2018
![image](https://user-images.githubusercontent.com/11457832/40274504-1bbb86a0-5c02-11e8-9bbd-ba33fde2f281.png)
New PCB using Wemos D1 mini, the smallest CNC/3D board ! not tested yet.

12-5-2018
Backplanner using grbl code, minimum corner using repetier code.

recode backlash algorithm
laser cutting co2 work (m3 S100 to turn on laser on G1)
fix timer problem on very slow speed (below 1)

i share my stm32 board too
![image](https://user-images.githubusercontent.com/11457832/39960100-9d05b922-5646-11e8-8dbd-913c726d806a.png)

5-2-2018

I change the feedrate acceleration algorithm, previously using mainloop that calculate f by adding it with a*dl , which dl is from 1/f, this cause problem cannot reach the target velocity.

Now it use inversesquare of accumulative value from acceleration, which is maybe costly on the mainloop, but more easy on path planner.

Also i have make the analog read better by able to pool more than 1 analogread and keep the result on buffer.

Also i plan to use more library, such as DIO2.
