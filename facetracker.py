#!/usr/bin/env python

# import the necessary packages
from scipy.interpolate import interp1d
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import cv2 as cv
import math
import RPi.GPIO as GPIO

# custom
import servo

count = 0
center = 90
pan  = center
tilt = center

# Load the model.
net = cv.dnn.readNet('face-detection-adas-0001.xml','face-detection-adas-0001.bin')

# Specify target device.
net.setPreferableTarget(cv.dnn.DNN_TARGET_MYRIAD) 
  
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 90
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.2)

#Center Mount
servo.pan(pan)
servo.tilt(tilt)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    
    image = frame.array
  
    #show center of camera circle
    #cv2.circle(image,(320, 240), threshold, (0,0,0), 2)    
   
   #show center of camera target
    cv2.line(image, (320, 280), (320, 200), (0,255,0), 1)
    cv2.line(image, (360, 240), (280, 240), (0,255,0), 1)
  
    # Prepare input blob and perform an inference.
    blob = cv.dnn.blobFromImage(image, size=(640, 480), ddepth=cv.CV_8U)
    net.setInput(blob)
    out = net.forward()
    # Draw detected faces on the frame.
    for detection in out.reshape(-1, 7):
        confidence = float(detection[2])
        xmin = int(detection[3] * image.shape[1])
        ymin = int(detection[4] * image.shape[0])
        xmax = int(detection[5] * image.shape[1])
        ymax = int(detection[6] * image.shape[0])
        if confidence > 0.6:
            cv.rectangle(image, (xmin, ymin), (xmax, ymax),  thickness=1, color=(0, 255, 0))
            
            #find center point of face 
            x_pixel = int((xmax + xmin)/2)
            y_pixel = int((ymax + ymin)/2)
            
            #map resolution to servo values
            x_map = interp1d([0,640],[180,0])
            y_map = interp1d([0,480],[180,0])
            #map area of rectangle as distance with arbitrary an value
            #z_map = interp1d([0,307200],[0,1000]) 
            

            #draw 2 lines through the center of the face
            cv2.line(image, (x_pixel, 0), (x_pixel, 480), (0,255,0), 2)
            cv2.line(image, (0, y_pixel), (640, y_pixel), (0,255,0), 2)

            #pixels to values the servo can use 
            x = int(x_map(x_pixel)) 
            y = int(y_map(y_pixel))
            
            #z axis "distance"
            #z = int(z_map(int((xmax - xmin) * (ymax - ymin))))
            
            #print('x:' +str(x_pixel)+',y:'+ str(y)+',z:'+str(z))
            
            #if the face is close to center draw a circle
            #if (x < 100 and x > 80) and (y < 100 and y > 80):
                #cv2.circle(image,(x_pixel, y_pixel), 30, (0,0,255),2)
            
            
            if x < center:
                pan -= 1
            elif x > center:
                pan += 1                                   
            servo.pan(pan)            
            
            if y > center:
                tilt -= 1
            elif y < center:
                tilt += 1               
            servo.tilt(tilt)
                           
            #print('pan: '+str(pan)+' , tilt: '+str(tilt))

    # show the frame
    cv2.imshow("Frame",image )
    key = cv2.waitKey(1) & 0xFF
 
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
 
    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
GPIO.cleanup()     
