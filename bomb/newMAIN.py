#!/usr/bin/env python3
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.nxtdevices import LightSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

# Create your objects here.
ev3 = EV3Brick()
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
spare = Motor(Port.D)
color = ColorSensor(Port.S4)
ir = Ev3devSensor(Port.S1)  
light = LightSensor(Port.S3)
gyro = GyroSensor(Port.S2)

#robot = DriveBase(left, right, wheel_diameter=5.08, axle_track=13.97)



THRESHOLD = 45  # Adjust this value based on your environment
DRIVE_SPEED = 150
TURN_RATE = 180
reflection = color.reflection()

def intake():
    intake.run(-750)  # Start the intake motor
    wait(275)
    intake.stop()

while (True):
    
    print(reflection)
    if line.reflection() > THRESHOLD*1.5:
        # Too far from the line (to the right), turn left
        left.run(DRIVE_SPEED - TURN_RATE)
        right.run(DRIVE_SPEED + TURN_RATE)
    elif line.reflection() < THRESHOLD*1.5:
        # Too far to the left, turn right
        left.run(DRIVE_SPEED + TURN_RATE)
        right.run(DRIVE_SPEED - TURN_RATE)
    else:
        # On the line, drive straight
        left.run(DRIVE_SPEED)
        right.run(DRIVE_SPEED)
    wait(20)
    reflection = color.reflection()