



import imutils

import numpy as np

import argparse

import cv2

import math





# define range of color yellow in HSV
lowerBound = np.array([23,100,100])
upperBound = np.array([30,255,255])

lowerWhite = np.array([0,0,0])
upperWhite = np.array([0,0,0])
#get video from videocaptue stream
cap = cv2.VideoCapture(1)  

while(cap.isOpened()):

    

    #Grab a frame from the screencast

 
    ret, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lowerBound, upperBound)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)
    #mask = res
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    #mask = cv2.medianBlur(mask,5)
    mask = cv2.GaussianBlur(mask, (5, 5), 0)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    mask_inv = abs(255-mask)
    
    res = cv2.bitwise_and(frame,frame, mask= mask)

   
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    #Contour detection

    contours, _ = cv2.findContours(mask, 1, 2)

    cv2.drawContours(mask, contours, -1, (128,255,0),3)

    for c in contours:

           rect = cv2.boundingRect(c)

           if rect[2] <10 or rect[3] <10: continue

           x,y,w,h = rect
	   if rect[2] >100 or rect[3] >100:

	   	   cv2.rectangle(frame, (x,y),(x+w,y+h),(255,255,255),2)

		   cv2.rectangle(mask, (x,y),(x+w,y+h),(255,255,255),2)
	
	
		   cv2.putText(mask, 'Person match', (x,y+h+10), 0,0.3,(255,255,255))
		   cv2.putText(frame, 'person match', (x,y+h+10), 0,0.5,(255,255,255))
	   else:
		   flag = 0
		   #check if two mid sized blobs are close
		   for c2 in contours:

           		rect2 = cv2.boundingRect(c2)
			x2,y2,w2,h2 = rect2
			if rect2[2] <25 or rect2[3] <25: continue
			if rect[2] <25 or rect[3] <25: continue
			if (math.sqrt(abs(rect2[2]-rect[2])**2 +  abs(rect2[3]-rect[3])**2) < 25):
			   cv2.rectangle(frame, (x,y),(x+w,y+h),(255,255,255),2)

		           cv2.rectangle(mask, (x,y),(x+w,y+h),(255,255,255),2)
			
	
		           cv2.putText(mask, 'Person match', (x,y+h+10), 0,0.3,(255,255,255))
		           cv2.putText(frame, 'person match', (x,y+h+10), 0,0.5,(255,255,255))	
			   flag = 1
		   if flag == 0:
		      cv2.rectangle(frame, (x,y),(x+w,y+h),(0,255,0),1)

		      cv2.rectangle(mask, (x,y),(x+w,y+h),(0,255,0),1)
	
		      cv2.putText(mask, 'Heat match', (x,y+h+10), 0,0.3,(0,255,0))
		      cv2.putText(frame, 'Heat match', (x,y+h+10), 0,0.3,(0,255,0))

    
    cv2.imshow('frame',frame) 
    cv2.imshow("Contour Mask", mask)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()
cv2.destroyAllWindows()



"""
    cv2.imshow('frame',frame)


    #Threshold and remove noise

    #frame = cv2.resize(frame, width = 500) #resizing

    mask = cv2.inRange(frame, lowerBound, upperBound)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    mask = cv2.GaussianBlur(mask, (3, 3), 0)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    mask_inv = abs(255-mask)



    #Contour detection

    contours, _ = cv2.findContours(mask, 1, 2)

    cv2.drawContours(mask, contours, -1, (128,255,0),3)

    for c in contours:

        rect = cv2.boundingRect(c)

        if rect[2] <40 or rect[3] <40: continue

        x,y,w,h = rect

        cv2.rectangle(mask, (x,y),(x+w,y+h),(128,255,0),2)

        cv2.putText(mask, 'person detected', (x+w+10,y+h), 0,0.3,(240,255,0))

    cv2.imshow("Contour Mask", mask)

                                             

    #normalize the frame, spread out values for display of original feed

    cv2.normalize(frame, frame, 0, 65535, cv2.NORM_MINMAX)

    np.right_shift(frame, 8, frame)

    frame = np.uint8(frame)

    

    #Overlay the mask on the frame

    matchedPixels = cv2.bitwise_and(frame, frame, mask = mask)



    #Display images

    cv2.imshow("Original(left) Masked(right)", np.hstack([frame, matchedPixels]))

    #cv2.imshow("original", frame)

    cv2.imshow("Thresholded", mask_inv)

    



    #Exit on "q"

    if cv2.waitKey(1) & 0xFF == ord("q"):

        break



#Cleanup


cap.release()
cv2.destroyAllWindows()
"""
