karyacontroller
I make this project to learn about the controller.

simple multi axis motor controller for use in cnc project.
This project is possible because other people work:
- Teacup (gcode parsing, sersendp_f)
- notgrbl (simple serial reading)
- google

its not 100% exact code, but i have read lots of their code, so my code will effected by them



feature i need to have in this software is: (* implemented)
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
- config files to set pins and parameter
- G0 and G1 can have different acceleration (travel vs feed/extrude)
- backward planner
- interrupt timing
- eeprom configuration
- heater E0


for now it still not use timer interrupt, just use micros() to control the timing..



Project files:
karyacontroller.project - just for editing in codelite
karyacontroller.ino - arduino ide
motion [motion.exe].PRJ - for quincy IDE, we can show graphics of the motion simulation here

Original code is in freebasic, you can check the old folder