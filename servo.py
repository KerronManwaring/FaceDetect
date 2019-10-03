#!/usr/bin/env python
import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(19, GPIO.OUT)
GPIO.output(19, GPIO.LOW)

GPIO.setup(13, GPIO.OUT)
GPIO.output(13, GPIO.LOW)


panServo = GPIO.PWM(13, 100) #pan
panServo.start(0)
tiltServo = GPIO.PWM(19, 100) #tilt
tiltServo.start(0)



def tilt(angle):
    #constraining the values to 0 to 180
    if angle > 180:
        angle = 180
    if angle < 0:
        angle = 0
        
    GPIO.output(13, 0)
    duty = float(angle) / 10.0 + 2.5
    tiltServo.ChangeDutyCycle(duty)


def pan(angle):
    #constraining the values to 0 to 180
    if angle > 180:
        angle = 180
    if angle < 0:
        angle = 0
    
    duty = float(angle) / 10.0 + 2.5
    panServo.ChangeDutyCycle(duty)
