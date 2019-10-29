# coding=utf-8

import cv2
import numpy as np

cap = cv2.VideoCapture(1)

def nothing(x):
    pass

# Creating a window for later use
cv2.namedWindow('HSV_thresh_tuning')

h_min = 0
h_max = 10
s_min = 135
s_max = 255
v_min = 0
v_max = 255

# Creating trackbars
cv2.createTrackbar('hMin', 'HSV_thresh_tuning', h_min, 180, nothing)
cv2.createTrackbar('hMax', 'HSV_thresh_tuning', h_max, 180, nothing)
cv2.createTrackbar('sMin', 'HSV_thresh_tuning', s_min, 255, nothing)
cv2.createTrackbar('sMax', 'HSV_thresh_tuning', s_max, 255, nothing)
cv2.createTrackbar('vMin', 'HSV_thresh_tuning', v_min, 255, nothing)
cv2.createTrackbar('vMax', 'HSV_thresh_tuning', v_max, 255, nothing)

while(1):

    _, frame = cap.read()
    dim = (800,600)
    resizedframe = cv2.resize(frame, dim) 
    #converting to HSV
    hsv = cv2.cvtColor(resizedframe,cv2.COLOR_BGR2HSV)

    # get info from track bar and apply to HSV_thresh_tuning
 
    h_min = cv2.getTrackbarPos('hMin','HSV_thresh_tuning')
    h_max = cv2.getTrackbarPos('hMax','HSV_thresh_tuning')
    s_min = cv2.getTrackbarPos('sMin','HSV_thresh_tuning')
    s_max = cv2.getTrackbarPos('sMax','HSV_thresh_tuning')
    v_min = cv2.getTrackbarPos('vMin','HSV_thresh_tuning')
    v_max = cv2.getTrackbarPos('vMax','HSV_thresh_tuning')

    # Normal masking algorithm
    lb1 = np.array([h_min, s_min, v_min])
    ub1 = np.array([h_max, s_max, v_max])
    lb2 = np.array([180-h_max, s_min, v_min])
    ub2 = np.array([180-h_min, s_max, v_max])

    mask1 = cv2.inRange(hsv, lb1, ub1)
    mask2 = cv2.inRange(hsv, lb2, ub2)
    mask = cv2.bitwise_or(mask1, mask2)
    masked = cv2.bitwise_and(resizedframe,resizedframe,mask = mask)

    cv2.imshow('HSV_thresh_tuning', masked)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()
