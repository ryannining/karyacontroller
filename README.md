karyacontroller

simple multi axis motor controller for use in cnc project.

I make this project to learn about the controller.

feature i need to have in this software is: (* implemented)
- per axis acceleration *
- per axis max feedrate *
- per axis jerk  *
- path planner (forward) *
- backward planner
- step per mm for each axis *
- interrupt timing
- implement motor stepper hardware layer
- implement endstop reading
- implement GCODE parser and processing
- implement flow control between motion gcode and non motion gcode


for now it still not use timer interrupt, just use micros() to control the timing..
This software also contain

Project files:
karyacontroller.project - just for editing in codelite
karyacontroller.ino - arduino ide
motion [motion.exe].PRJ - for quincy IDE, we can show graphics of the motion simulation here

Original code is in freebasic, you can check the old folder