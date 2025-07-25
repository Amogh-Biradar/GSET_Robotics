#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor
from pybricks.nxtdevices import LightSensor


ev3 = EV3Brick()
left = Motor(Port.B)
right = Motor(Port.D)
m = Motor(Port.C)
color = ColorSensor(Port.S4)
ir = Ev3devSensor(Port.S1)  
line = LightSensor(Port.S3)
gyro = GyroSensor(Port.S2)

def pid_gyro_turn(target_angle, speed=100, kp=1.2):
    """Turn robot in place by a specified angle (degrees) using proportional control for smoother, more accurate turns."""
    gyro.reset_angle(0)
    error = target_angle - gyro.angle()
    while abs(error) > 1:
        turn_rate = kp * error
        # Clamp turn_rate to max speed
        if turn_rate > speed:
            turn_rate = speed
        elif turn_rate < -speed:
            turn_rate = -speed
        robot.drive(0, turn_rate)
        wait(10)
        error = target_angle - gyro.angle()
    robot.stop()


def drive_straight_gyro(distance_mm, speed=100):
    """Drives forward a given distance (mm) without gyro correction."""
    robot.reset()
    robot_distance = 0
    while abs(robot_distance) < abs(distance_mm):
        robot.drive(speed if distance_mm > 0 else -speed, 0)
        robot_distance = robot.distance()
    robot.stop()

# --- House Navigation Loop ---
def scan_ir_during_gyro_turn():
    """Turn 360 degrees, scan IR sensor, and return the max value detected."""
    gyro.reset_angle(0)
    integral = 0
    last_error = 0
    start_time = ev3.control.timer.now()
    target_angle = 360
    max_ir = 0
    while True:
        current_angle = gyro.angle()
        error = target_angle - current_angle
        integral += error
        derivative = error - last_error
        turn_rate = 100  # Use a moderate constant turn rate for smooth scan
        robot.drive(0, turn_rate)
        last_error = error
        # Scan IR sensor
        ir_val = ir.distance()
        if ir_val is not None and ir_val > max_ir:
            max_ir = ir_val
        ev3.wait(20)
        if current_angle >= target_angle:
            break
        if ev3.control.timer.now() - start_time > 4000:
            break
    robot.stop()
    return max_ir

def checkIR():
    x = ir.read("AC")
    if(x[0] != 0):
        return True
    return False
    

#robot = DriveBase(left, right, wheel_diameter=5.08, axle_track=13.97)

robot = DriveBase(left, right, wheel_diameter=54, axle_track=146.05)

THRESHOLD = 45  # Adjust this value based on your environment
DRIVE_SPEED = 150
TURN_RATE = 170
  # mm/s
PROPORTIONAL_GAIN = 1.8
reflection = color.rgb()


start_timer = StopWatch()




def intake(): #TEST
    m.run(250)  # Start the intake motor
    wait(1000)
    m.hold()

red = color.rgb()[0]
if red == 0:
    red = 1
blue = color.rgb()[2]
if blue == 0:
    blue = 1


blue_detected = False
victimCount = 0
gyro.reset_angle(0)
starting_angle = gyro.angle()




while (True):
    print(color.rgb())
    
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
    
    
    ev3.screen.clear()
    ev3.screen.draw_text(50, 60, blue/red)
    if red / blue  > 2 :
        robot.stop()
        # --- At red line: correct heading ---
        current_angle = gyro.angle()
        angle_diff = current_angle - starting_angle
        # Normalize to [-180, 180] for shortest turn
        if angle_diff > 180:
            angle_diff -= 360
        elif angle_diff < -180:
            angle_diff += 360
        pid_gyro_turn(-angle_diff)  # Negative to correct back to starting heading
        break
    #     break

    # # Check for blue line with the right-side color sensor
    if blue / red  > 3:
        if not blue_detected:
            robot.stop()
            blue_detected = True
            victimCount += 1
            ev3.speaker.beep(frequency=1000, duration=300)  # Stop for 3 seconds
            drive_straight_gyro(42)
        #If blue is still detected, do not stop again
    else:
        blue_detected = False

    
    wait(20)

    red = color.rgb()[0]
    if red == 0:
        red = 1
    blue = color.rgb()[2]
    if blue == 0:
        blue = 1
wait(50)
    
drive_straight_gyro(250)
pid_gyro_turn(-90)
drive_straight_gyro(510)
ev3.speaker.beep()
if checkIR():
    
    drive_straight_gyro(250)
    pid_gyro_turn(88)
    drive_straight_gyro(415)
    intake()
    drive_straight_gyro(-385)
    pid_gyro_turn(-90)
    drive_straight_gyro(-560)
    pid_gyro_turn(90)
    drive_straight_gyro(-250)
else:
    #Room 2
    drive_straight_gyro(-205)
    pid_gyro_turn(90)
    drive_straight_gyro(120)
    ev3.speaker.beep()
    if checkIR(): #check if it runs properly the arm
        drive_straight_gyro(200)
        pid_gyro_turn(90)
        drive_straight_gyro(215)
        intake()
        drive_straight_gyro(-330)
        pid_gyro_turn(-90)
        drive_straight_gyro(-320)
        pid_gyro_turn(-90)
        drive_straight_gyro(-240)
        pid_gyro_turn(90)
        drive_straight_gyro(-320)
        spare.run(250)
        wait(1000)
        spare.hold()
    else:
        drive_straight_gyro(-70)
        pid_gyro_turn(-85)
        drive_straight_gyro(-585)
        pid_gyro_turn(90)
        ev3.speaker.beep()
        drive_straight_gyro(370)
        intake()
        drive_straight_gyro(-370)
        pid_gyro_turn(-93)
        drive_straight_gyro(300, speed=250)
        pid_gyro_turn(-90)
        drive_straight_gyro(250, speed=250)



# Write your program here.
