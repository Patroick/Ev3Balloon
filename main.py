#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, 
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.nxtdevices import ColorSensor, LightSensor

ev3 = EV3Brick()

motorRight = Motor(Port.A)
motorLeft = Motor(Port.B)
motorNeedle = Motor(Port.C)
UltrasonicSensor = UltrasonicSensor(Port.S3)
TouchSensorRight = TouchSensor(Port.S1)
TouchSensorBack = TouchSensor(Port.S4)
LightSensor = LightSensor(Port.S2)

ev3.speaker.beep()

Balloons = 1;

robot = DriveBase(motorLeft, motorRight, wheel_diameter=28, axle_track=187)

def destroy():
    print(motorNeedle.angle())
    motorNeedle.run_target(1000, -140)
    motorNeedle.run_target(500, 0)

while True:
    if Balloons == 0:
        print('Remaining Balloons: ', Balloons);
        print('Destroyed all Balloons: ', Balloons);
        break
    destroy();
    print('Remaining Balloons: ', Balloons);
    Balloons = Balloons - 1
    print(LightSensor.reflection())


        
    