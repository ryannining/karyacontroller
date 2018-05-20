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
- Per motor step using float speed ramping acceleration (smooth ) *
- per axis acceleration *
- per axis max feedrate *
- per axis jerk  *
- path planner (backward and forward) *
- step per mm for each axis *
- motor stepper hardware layer using GPIO and ShiftRegister *
- GCODE parser and processing *
- 4 axis X Y Z E *
- eeprom configuration (step/mm, accel, travel accel, jerk, max axis) *
- implement endstop reading (xmin xmax , etc)*
- Endstop configuration using EEPROM *
- heater with PID E0 *
- Async temp reading *
- config files contain board definition to set pins and parameter *
- G0 and G1 can have different acceleration (travel vs feed/extrude) *
- backlash compensation *
- Config for inverted motor and endstop *
- interrupt timing * (for AVR, ESP8266, STM32F103)

## MCU
- Nano V3 328p * tested
- Mega 2560 *
- Wemos D1 *
- STM32 bluepill * tested

## EEPROM
Yes, its support EEPROM and modification in repetier host. 

## WIFI
Still work in progress, especially for ESP8266

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
