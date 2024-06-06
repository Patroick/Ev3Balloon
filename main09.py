#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.nxtdevices import LightSensor
import time

left_motor  = Motor(Port.B)
right_motor = Motor(Port.A)
pop_motor   = Motor(Port.C)
front_touch  = TouchSensor(Port.S4)
abyss_touch = TouchSensor(Port.S1)
light_sensor = LightSensor(Port.S2)
usonic_sensor = UltrasonicSensor(Port.S3)

robot = DriveBase(left_motor, right_motor, 56, 114)

# Create your objects here.
ev3 = EV3Brick()

# Write your program here.
# beep = program started
ev3.speaker.beep()

#---- configuration ----
# these variables are for finetuning the robots behavior
# mm/s
drive_velocity = 170

# backwards-distance on no contact in mm (10 cm = 3.5cm with tracks)
reverse_distance = 200

# rotation after reversal whenever edge is hit in ° (~270° = 90°; 1° wheels = 3° tracks)
adjustment_rotation = 90 # 30° rotation
adjustment_factor = 2.8

# checking rate in ms (1000 = 1 second)
loop_delay = 1

# the distance the robot should drive before checking if he finds any color
distance_before_check = 500

# in seconds
turn_stabilization_time = 0.3

# RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, VIOLET, MAGENTA, can also be set via Color(hue, saturation, brightness)
color_needed = Color.RED

# ---- runtime vars ----
# should not be manipulated by the user

distance_gone = 0

bloonReflection = 0
reflectionDistance = 0
ambient = 0
endReflection = 27
popRadius = 60

bloonSighted = False

baloon_count = 1

runs = 0


def GetBloonColor(turnAngle):
    global bloonReflection
    global reflectionDistance
    global ambient
    robot.turn(turnAngle)
    bloonReflection    = light_sensor.reflection()
    ambient = light_sensor.ambient()
    print(bloonReflection)
    reflectionDistance = usonic_sensor.distance()
    robot.turn(turnAngle * -1)

def isNoBloonInFront():
    reliabilityDeltaLight = 5
    reflection = light_sensor.reflection() + ambient - light_sensor.ambient()
    return not ((bloonReflection - 6 <= reflection <= bloonReflection + 6) and (usonic_sensor.distance() < 150))

# init color + turn to front
GetBloonColor(0)
robot.straight(100)

robot.turn(180 * adjustment_factor)


# drive to edge and a little back
while not front_touch.pressed():
    robot.drive(-drive_velocity, 0)
robot.straight(100)

robot.turn(-90 * (adjustment_factor + 0.1))

while baloon_count > 0 and runs < 2:
    while abyss_touch.pressed() and isNoBloonInFront() and not front_touch.pressed():
        robot.drive(drive_velocity, 0)
    if not abyss_touch.pressed() or front_touch.pressed():
        drive_velocity = drive_velocity * -1
        robot.drive(drive_velocity, 0)
        runs += 1
        wait(1000)
    else:
        robot.stop()
        # needle arm gets rammed into bloon till usonic sensor no longer senses baloon
        while (usonic_sensor.distance() <= reflectionDistance):
            pop_motor.run_target(999999, -popRadius)
            pop_motor.run_target(999999, 0)
            wait(300)
        baloon_count -= 1
        runs = 0
runs = 0
while not (endReflection - 3 <= light_sensor.reflection() <= endReflection + 3 and usonic_sensor < 300) and runs < 2:
    while abyss_touch.pressed() and not front_touch.pressed() and (light_sensor.reflection() != 0 or usonic_sensor.distance() > 500):
        robot.drive(drive_velocity, 0)
    if not abyss_touch.pressed() or front_touch.pressed():
        drive_velocity = drive_velocity * -1
        robot.drive(drive_velocity, 0)
        runs += 1
        wait(1000)
robot.stop()

if runs < 2:
    pop_motor.run_target(999999, -popRadius)
    pop_motor.run_target(999999, 0)
    




# Popverhalten
#while True:
#    while left_touch.pressed() and right_touch.pressed() and usonic_sensor.distance() >= 10 and usonic_sensor.distance() < 2500:
#            robot.drive(drive_velocity, 0)
#    if usonic_sensor.distance() < 10 or usonic_sensor.distance() >= 2500:
#        robot.straight(-250)
#    ReverseToNewPosition(reverseTurn, reverseDistance)
    #while usonic_sensor.distance() > 1000:
    #    robot.straight(300)
    #    ReverseToNewPosition(reverseTurn, reverseDistance)
    #while left_touch.pressed() and right_touch.pressed() and usonic_sensor.distance() >= 10 and usonic_sensor.distance() < 2500:
    #    robot.drive(drive_velocity, 0)

# Test Color Detection
#while True:
#    print('Distanzunterschied: ', usonic_sensor.distance() - reflectionDistance)
#    if (not isNoBloonInFront()):
#        print('actual: ', light_sensor.reflection())
#        print('ambient: ', light_sensor.ambient())
#        print('expected: ', bloonReflection)
#        print('bloonFound')
#    else:
#        print('actual: ', light_sensor.reflection())
#        print('ambient: ', light_sensor.ambient())
#        print('expected: ', bloonReflection)
#        print('bloonNotFound')
#    wait(1000)

# Edge Detection
#while True:
#    while left_touch.pressed() and right_touch.pressed():
#        robot.drive(drive_velocity, 0)