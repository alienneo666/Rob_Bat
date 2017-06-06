#! /usr/bin/env python

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import serial

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 50
camera.hflip = True

rawCapture = PiRGBArray(camera, size=(640, 480))
 
time.sleep(0.1)
 
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        image = frame.array
        blur = cv2.blur(image, (3,3))

        lower = np.array([76,31,4],dtype="uint8")
        upper = np.array([210,90,70], dtype="uint8")

        thresh = cv2.inRange(blur, lower, upper)
        thresh2 = thresh.copy()

        image, contours,hierarchy = cv2.findContours(thresh,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        best_cnt = 1
        for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > max_area:
                        max_area = area
                        best_cnt = cnt

        M = cv2.moments(best_cnt)
        cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
     
        #if best_cnt>1:
        #circulo exterior e interior que marca el centro en otro color
        cv2.circle(blur,(cx,cy),12,(45,235,252),2) 
        cv2.circle(blur,(cx,cy),10,(0,0,255),-1)

        #capturo las coordenadas y las imprimo en pantalla
        print("x= ")
        print(cx)
        print(" y=  ")
        print(cy)
        # show the frame
        cv2.imshow("Frame", blur)
        #cv2.imshow('thresh',thresh2)
        key = cv2.waitKey(1) & 0xFF
        if cx > 320:
            arduino.write(b'SH'+str(cx-320))  
	# clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        
	# if the `q` key was pressed, break from the loop
        if key == ord("q"):
        	break
