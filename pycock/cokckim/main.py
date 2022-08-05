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
baseL = Motor(Port.C, Direction.CLOCKWISE)
baseR = Motor(Port.B, Direction.COUNTERCLOCKWISE)
armL = Motor(Port.A, Direction.COUNTERCLOCKWISE)
# water = Motor(Port.D, Direction.COUNTERCLOCKWISE)
base = DriveBase(baseL,baseR,97,68)

gyro = GyroSensor(Port.S4)
colourMid = ColorSensor(Port.S1)
colourLeft = nxtColorSensor(Port.S2)
colourRight = nxtColorSensor(Port.S3)

colourMidVal, colourLeftVal, colourRightVal, motorLVal, motorRVal, colourMidReflect = colourMid.rgb(), colourLeft.reflection(), colourRight.reflection(), baseL.angle(), baseR.angle(), colourMid.reflection()

lightVal = 0

rgbTH = [r, r1, g, g1, b, b1] = [0, 0, 0, 0, 0, 0]

Kp = 0
Kd = 0

GKp = 1.7

RGBTH = (12,12,12)

TH = 220
count = 0

person1 = ""
person2 = ""
chemical = ""

objectAlrPresent = False

stopwatch = StopWatch()

def handler(signum, frame):
    print("Forever is over")
    raise Exception("end of time")

def readVal():
    global gyroVal, colourMidVal, colourLeftVal, colourRightVal, motorLVal, motorRVal, lightVal, colourMidReflect
    gyroVal = gyro.angle()
    colourMidVal = colourMid.rgb()
    lightVal = colourMidVal[0] + colourMidVal[1] + colourMidVal[2]
    colourLeftVal = colourLeft.reflection()
    colourRightVal = colourRight.reflection()
    motorLVal = baseL.angle()
    motorRVal = baseR.angle()
    colourMidReflect = colourMid.reflection()

checkVals = lambda colourmidVal, RGBTH: not all(colourmidVal[i] < RGBTH[i] for i in range(len(colourmidVal)))

def checkValues(colourMidVal, RGBTH):
    if checkVals(colourMidVal, RGBTH):
        return False
    else:
        return True


def colourDetect(R, R1, G, G1, B, B1):
    rgbTH = [R, R1, G, G1, B, B1]
    while True:
        if rgbTH[0] > colourMidVal[0] == rgbTH[2] > colourMidVal[1] == rgbTH[4] > colourMidVal[2] == rgbTH[1] < colourmidVal[0] == rgbTH[3] < colourMidVal[1] == rgbTH[5] < colourMidVal[2]:
            print("true")
            return False
            
        else:
            print("false")
            return True
            break

def angleDetect(angle):
    readVal()
    if abs(baseL.angle()) <= angle and abs(baseR.angle()) <= angle:
        return True
    else:
        return False


# def detect():
#     while True:
#         print("red: {} green: {} blue: {}".format(*colourMid.rgb()))

class motorEn:
    def straight(speed, param): 

        while param():
            readVal()

            Error = motorLVal - motorRVal
            Correction = GKp * Error * -1
            result = min(Correction, 1050)
            
            baseL.run(speed - result)
            baseR.run(speed + result)
            
            # difference = motorLVal - motorRVal
            # pastError = pastError + difference
            # print("L motor:", motorLVal, "R motor:", motorRVal, "Error:", Error, "Correction:", Correction, "Difference", difference)
        baseL.brake()
        baseR.brake()

    def turn(speed, angle):
        stopwatch.reset()
        baseL.reset_angle(0)
        baseR.reset_angle(0)

        while motorRVal != angle and stopwatch.time() < 5000:
            readVal()
            Error = angle - motorRVal
            Correction = 0.5 * Error
            result = min(Correction, 1050)
            baseL.run(-result)
            baseR.run(result)
        baseL.brake()
        baseR.brake()
        print("End of turn")
def PD(speed, args):
    while args:
        readVal()
        Error = lightVal - TH
        Correction = Kp * Error + Kd * (Error - lastError)
        result = min(Correction, 1050)
        baseL.run(speed - result)
        baseR.run(speed + result)
        lastError = Error
    baseL.brake()
    baseR.brake()

def locatePerson(sector):
    global person1, person2
    if person1 != "":
        person2 = sector
    else:
        person1 = sector

def senseObject(sector):
    global person1, person2, chemical, objectAlrPresent
    if colourMidVal[0] and colourMidVal[1] and colourMidVal[2] > 70:
        print("Person")
        locatePerson(sector)
        return True
    elif colourMidVal[0] and colourMidVal[1] and colourMidVal[2] < 30:
        print("Chemical")
        chemical = sector
        return True
        #Grab chemical

    elif colourMidVal[0] > 70 and colourMidVal[1] and colourMidVal[2] < 40:
        print("Fire")
        return True
        #Drop water
    else:
        print("Nothing")
        return False

def printVals():
    while True:
        print("red: {} green: {} blue: {}".format(*colourMid.rgb()))
        print("light: {}".format(lightVal))
        print("motorL: {} motorR: {}".format(motorLVal, motorRVal))
        print("gyro: {}".format(gyro.angle()))
        print("colourMidReflect: {}".format(colourMidReflect))
        print("")
        wait(1000)

# Write your program here.
gyro.reset_angle(0)
baseL.reset_angle(0)
baseR.reset_angle(0)
base.settings(-400)



print("Start")
readVal()

# Gyro.straight(-500, 0, lambda: angleDetect(900))

# print("-")

# Gyro.straight(-250, 0, lambda: checkVals(colourMidVal, RGBTH))
# print(colourMidVal)
# print("Black Line")

# wait(300)

# print(gyro.angle())

# Gyro.turn(500, -90)

# wait(300)

# #Wallbang
# base.drive(-500)
# wait(1000)
# base.stop()

# gyro.reset_angle(0)

print(motorLVal, motorRVal)
motorEn.turn(500, -90)
print(motorLVal, motorRVal)


# printVals()
print("End")



# #Turn to Line
# while count < 2:
#     readVal()
#     while checkVals:
#         baseL.run(50)
#         readVal()
#     count += 1

# #Drive to blue 
# PD(50, colourMidVal[2] < 80 and colourMidVal[0] and colourMidVal[1] > 80)

#Sense object 
# Gyro.straight(50, 90, 0) #Last arguement to be filled with unnamed value (May be no objects at all, need to add conditional statement)
# readVal()
# senseObject("Blue")

# #Drive to Green
# Gyro.straight(50, 90, colourMidVal[1] > 70 and colourMidVal[0] and colourMidVal[2] < 70)

# #Sense Object (Assume that able to sense object when midSensor detects green)
# readVal()
# senseObject("Green")

# #Drive and sense other object
# if senseObject("Green") == False:
#     Gyro.straight(50, 90, 0) #Last arguement to be filled with unnamed value (May be no objects at all, need to add conditional statement)
#     readVal()
#     senseObject("Green")
#     Gyro.turn(50, 0)
# else:
#     base.straight(350)
#     Gyro.turn(50, 0)

# #Wallbang
# base.drive(-100, 0)
# wait(1000)
# base.stop()
# Gyro.reset_angle(0)

# #Drive to Yellow
# Gyro.straight(50, 0, colourMidVal[0] and colourMidVal[1] > 70 and colourMidVal[2] < 70)

# #Sense Object
# readVal()
# senseObject("Yellow")

# #Drive and sense other object
# Gyro.straight(50, 0, 0) #Last arguement to be filled with unnamed value (May be no objects at all, need to add conditional statement)
# readVal()
# senseObject("Yellow")

# #Drive to Red
# Gyro.straight(50, 0, colourMidVal[0] > 70 and colourMidVal[1] and colourMidVal[2] < 70)

# #Sense Object
# readVal()
# senseObject("Red")

# #Drive and sense other object
# if senseObject("Red") == False:
#     Gyro.straight(50, 0, 0) #Last arguement to be filled with unnamed value (May be no objects at all, need to add conditional statement)
#     readVal()
#     senseObject("Red")
#     Gyro.turn(50, -90)
# else:
#     base.straight(350)
#     Gyro.turn(50, -90)

# #Wallbang
# base.drive(-100, 0)
# wait(1000)
# base.stop()
# Gyro.reset_angle(0)

# #Drive to Brown
# Gyro.straight(50, 0, colourMidVal[0] and colourMidVal[1] < 70 and colourMidVal[2] > 70) #Update brown colour rgb as necessary

# #Sense Object
# readVal()
# senseObject("Brown")

# #Drive and sense other object
# if senseObject("Brown") == False:
#     Gyro.straight(50, 0, 0) #Last arguement to be filled with unnamed value (May be no objects at all, need to add conditional statement)
#     readVal()
#     senseObject("Brown")
    
# else:
#     base.straight(350)

# #Cross the wall and reach end point
# Gyro.straight(50, 0, 0)
