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
from pybricks.hubs import EV3Brick 
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,                                  
InfraredSensor, UltrasonicSensor, GyroSensor) 
from pybricks.parameters import Port, Stop, Direction, Button, Color 
from pybricks.tools import wait, StopWatch, DataLog 
from pybricks.robotics import DriveBase 
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher. # Click "Open user guide" on the EV3 extension tab for more information.
ev3 = EV3Brick()
left = Motor(Port.C)
right = Motor(Port.B)
flip = Motor(Port.A)
ultra = UltrasonicSensor(Port.S1)
color = ColorSensor(Port.S2)
line = LightSensor(Port.S3)
robot = DriveBase(left, right, wheel_diameter=54, axle_track=200)

ev3.light.on(Color.YELLOW) 

while Button.CENTER not in ev3.buttons.pressed():
    wait(10) 
    
ev3.light.on(Color.GREEN) 
ev3.speaker.beep() 
wait(5000)

#PUT YOUR CODE HERE  Delete mine

# Advanced Sumo Robot with Line Detection
ev3.light.on(Color.RED)  # Battle mode indicator


#Robot configuration optimized for 36" ring with 1" white border
SEARCH_SPEED = 500      # Slower search for precise control in small ring
ATTACK_SPEED = 1000      # Speed when attacking
TURN_SPEED = 1000        # Speed when turning
DETECTION_DISTANCE = 300 # Shorter detection range for 36" ring (cm)
COLOR_WHITE_THRESHOLD = 10    # Color sensor threshold for gloss white paint
LIGHT_WHITE_THRESHOLD = 40    # Light sensor threshold for gloss white paint (different scale)

def check_line_sensors():
    """Check if we're approaching the white boundary line"""
    color_reading = color.rgb()  # Color sensor reading
    light_reading = line.reflection()   # Light sensor reading
    
    # If either sensor detects white (high reflection), we're near the edge
    # Different thresholds for different sensor types on gloss white paint
    color_detects_white = color_reading[0] > COLOR_WHITE_THRESHOLD or color_reading[1] > COLOR_WHITE_THRESHOLD or color_reading[2] > COLOR_WHITE_THRESHOLD
    light_detects_white = light_reading > LIGHT_WHITE_THRESHOLD
    
    return color_detects_white or light_detects_white

def avoid_line():
    """Emergency line avoidance maneuver - FAST and unpredictable"""
    ev3.speaker.beep(2000, 100)  # Quick warning beep
    ev3.light.on(Color.YELLOW)   # Warning color
    
    # IMMEDIATE sharp turn away from line (no backing up first)
    
    # Random sharp turn direction - 120-180 degrees for maximum evasion
    import random
    robot.drive(-1000,0)
    wait(3000)
    turn_angle = random.randint(180, 215)  # Smaller turns for 36" ring
    turn_direction = random.choice([-1, 1])
    
    # Execute rapid turn at maximum speed
    robot.turn(turn_angle * turn_direction)
    
    # Quick forward burst to get away from edge immediately
    robot.drive(ATTACK_SPEED, 0)
    wait(750)  # Shorter burst for small ring
    
    robot.stop()
    ev3.light.on(Color.RED)  # Back to battle mode

# Main sumo battle loop - runs indefinitely
while True:
    print("color sensor:", color.rgb())  # Debug: print color sensor readings
    print("light sensor:", line.reflection())  # Debug: print light sensor readings
    # robot.drive(1000, 90)  # Default search speed
    # wait(150)
    # robot.drive(1000, 0)  # Default search speed
    # wait(300)
    # robot.drive(1000, 90)  # Default search speed
    # wait(150)

    robot.drive(1000, 120)  # Default search pattern - turn right
    wait(2000)
    
    if check_line_sensors():
        avoid_line()
        continue

    distance = ultra.distance()


    if ultra.distance() <= DETECTION_DISTANCE:
        robot.stop()
        ev3.light.on(Color.RED)
        ev3.speaker.beep(800, 100)  # Attack sound
        
        # Deploy flipper mechanism aggressively
        flip.run(800)
        wait(250)
        flip.stop()
        
        # Charge forward at full speed, but check line sensors continuously
        attack_count = 0
        while attack_count < 60:  # Attack for up to 4.5 seconds
            robot.drive(ATTACK_SPEED, 0)
            wait(300)  # Quick checks during attack
            if check_line_sensors():
                avoid_line()
                continue
            attack_count += 1

            if ultra.distance() < DETECTION_DISTANCE / 4:
                flip.run(-800)
                wait(225)
                flip.stop()
            
            # Update distance during attack
            if ultra.distance() > DETECTION_DISTANCE * 2:
                break  # Opponent pushed away, stop attacking
        
        # Stop attack if line detected

        robot.stop()
        
        # Retract flipper
        flip.run(800)
        wait(225)
        flip.stop()
        avoid_line()  # Check for line again after attack
        
        # Brief pause before next action
        wait(200)

    # PRIORITY 1: Check line sensors first (safety)
      # Skip to next loop iteration
    
    # Get ultrasonic distance reading
    
    
    # PRIORITY 2: If opponent detected within range, ATTACK!
    
        
    # PRIORITY 3: No opponent detected, search pattern
    else:
        ev3.light.on(Color.ORANGE)
        
        # Ensure flipper is retracted during search mode
        flip.run(800)
        wait(225)
        flip.stop()
        
        # Advanced search: move in expanding spiral while checking boundaries
        search_count = 0
        while search_count < 8 and not check_line_sensors():  # Shorter search for small ring
            # Spiral search pattern - forward with gradual turn
            turn_rate = 40 + (search_count * 8)  # Tighter spiral for 36" ring
            robot.turn(turn_rate)
            wait(100)
            search_count += 1
            
            # Quick opponent check during search
            if ultra.distance() <= DETECTION_DISTANCE:
                break
        
        # If we completed the search without finding anything, try a different pattern
        if search_count >= 8:
            # Quick spin to scan for opponents - smaller angle for small ring
            robot.turn(30)
            wait(150)
    
    # Micro-pause for sensor stability
    wait(30)
