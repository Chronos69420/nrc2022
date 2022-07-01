#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile
from pybricks.nxtdevices import ColorSensor as nxtColorSensor

# Create your objects here.
ev3 = EV3Brick()
baseL = Motor(Port.C, Direction.COUNTERCLOCKWISE)
baseR = Motor(Port.B, Direction.CLOCKWISE)
armL = Motor(Port.A, Direction.COUNTERCLOCKWISE)
armR = Motor(Port.D, Direction.CLOCKWISE)
base = DriveBase(baseL,baseR,97,68)

gyro = GyroSensor(Port.S1)
colourMid = ColorSensor(Port.S4)
colourLeft = nxtColorSensor(Port.S2)
colourRight = nxtColorSensor(Port.S3)

Kp = 0
Kd = 0

GKp = 5

RGBTH = 0

def readVal():
    global gyroVal, colourMidVal, colourLeftVal, colourRightVal, motorLVal, motorRVal
    gyroVal = gyro.angle()
    colourMidVal = colourMid.reflection()
    colourLeftVal = colourLeft.reflection()
    colourRightVal = colourRight.reflection()
    motorLVal = baseL.angle()
    motorRVal = baseR.angle()

class Gyro:
    def straight(speed, args): 
        while args():
            Error = 0 - gyro.angle()
            wait(1000)
            print("The fucking gyro angle is: ", gyro.angle())
            print("The fucking error is: ", Error)
            Correction = GKp * Error
            result = min(Correction, 1050)
            baseL.run(speed - result)
            baseR.run(speed + result)
    def turn(speed, angle):
        while gyro.angle() != angle:
            Error = angle - gyro.angle()
            wait(1000)
            print("The fucking gyro angle is: ", gyro.angle())
            print("The fucking error is: ", Error)
            Correction = GKp * Error
            result = min(Correction, 1050)
            baseL.run(speed - result)
            baseR.run(speed + result)

            

# Write your program here.

Gyro.turn(180, 90)


