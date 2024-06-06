#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.nxtdevices import ColorSensor, LightSensor

# Variables that should be set by the user #################################################################
############################################################################################################

ev3 = EV3Brick()

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
balloon_motor = Motor(Port.B)
ultrasonic_sensor = UltrasonicSensor(Port.S4)
light_sensor = LightSensor(Port.S2)
front_touch_sensor = TouchSensor(Port.S3)
bottom_touch_sensor = TouchSensor(Port.S1)

robot = DriveBase(left_motor, right_motor, wheel_diameter=28, axle_track=165)

Balloons = 3;

# Functions ################################################################################################
############################################################################################################

def initBalloonColor():
    global needed_light_reflection
    global init_light_ambient
    needed_light_reflection = light_sensor.reflection()
    init_light_ambient = light_sensor.ambient()
    print('Reflection: ', needed_light_reflection)
    print('Ambient: ', init_light_ambient)

def driveToBar():
    while not front_touch_sensor.pressed():
        robot.drive(50, 0)
    
    robot.straight(-15)
    robot.turn(80)
    robot.straight(100)
    robot.turn(10)

def relReflection():
    return needed_light_reflection + init_light_ambient - light_sensor.ambient()

def destroy():
    print(balloon_motor.angle())
    balloon_motor.run_target(1000, -140)
    balloon_motor.run_target(500, 0)

# Main Program #############################################################################################
############################################################################################################

initBalloonColor()
driveToBar()

while relReflection() - 8 >= light_sensor.reflection()  <= relReflection() + 8:
    robot.drive(50, 0)

robot.stop()

print('Found Balloon')
print('Reflection: ', light_sensor.reflection())
print('Ambient: ', light_sensor.ambient())
ev3.speaker.beep()


        
    