# coding=utf-8

import cv2
import numpy as np

# Uncomment if you want to acquire from video
# cap = cv2.VideoCapture(1)

def nothing(x):
    pass

# Creating a window for later use
cv2.namedWindow('HLS_thresh_tuning')

h_min = 0
h_max = 255
l_min = 150
l_max = 255
s_min = 0
s_max = 70

# Creating trackbars
cv2.createTrackbar('hMin', 'HLS_thresh_tuning', h_min, 180, nothing)
cv2.createTrackbar('hMax', 'HLS_thresh_tuning', h_max, 180, nothing)
cv2.createTrackbar('lMin', 'HLS_thresh_tuning', l_min, 255, nothing)
cv2.createTrackbar('lMax', 'HLS_thresh_tuning', l_max, 255, nothing)
cv2.createTrackbar('sMin', 'HLS_thresh_tuning', s_min, 255, nothing)
cv2.createTrackbar('sMax', 'HLS_thresh_tuning', s_max, 255, nothing)

while(1):

    # Video
    '''
    _, frame = cap.read()
    '''

    # Picture
    frame = cv2.imread("white_ball/wb7.jpg");

    dim = (800,600)
    resizedframe = cv2.resize(frame, dim) 
    #converting to HLS
    hls = cv2.cvtColor(resizedframe,cv2.COLOR_BGR2HLS)

    # get info from track bar and apply to HLS_thresh_tuning
 
    h_min = cv2.getTrackbarPos('hMin','HLS_thresh_tuning')
    h_max = cv2.getTrackbarPos('hMax','HLS_thresh_tuning')
    l_min = cv2.getTrackbarPos('lMin','HLS_thresh_tuning')
    l_max = cv2.getTrackbarPos('lMax','HLS_thresh_tuning')
    s_min = cv2.getTrackbarPos('sMin','HLS_thresh_tuning')
    s_max = cv2.getTrackbarPos('sMax','HLS_thresh_tuning')

    # Normal masking algorithm
    lb = np.array([h_min, l_min, s_min])
    ub = np.array([h_max, l_max, s_max])

    mask = cv2.inRange(hls, lb, ub)
    masked = cv2.bitwise_and(resizedframe,resizedframe,mask = mask)

    cv2.imshow('HLS_thresh_tuning', masked)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()
