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

for now it still not use timer interrupt, just use micros() to control the timing..
This software also contain
