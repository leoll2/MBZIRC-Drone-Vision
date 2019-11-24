# coding=utf-8

import cv2
import numpy as np

# Uncomment if you want to use camera instead of calib pictures
# cap = cv2.VideoCapture(0)

def nothing(x):
    pass

# Creating a window for later use
cv2.namedWindow('LAB_thresh_tuning')

l_min = 0
l_max = 255
a_min = 160
a_max = 255
b_min = 130
b_max = 255

# Creating trackbars
cv2.createTrackbar('lMin', 'LAB_thresh_tuning', l_min, 255, nothing)
cv2.createTrackbar('lMax', 'LAB_thresh_tuning', l_max, 255, nothing)
cv2.createTrackbar('aMin', 'LAB_thresh_tuning', a_min, 255, nothing)
cv2.createTrackbar('aMax', 'LAB_thresh_tuning', a_max, 255, nothing)
cv2.createTrackbar('bMin', 'LAB_thresh_tuning', b_min, 255, nothing)
cv2.createTrackbar('bMax', 'LAB_thresh_tuning', b_max, 255, nothing)


while True:
    # Read from image
    frame = cv2.imread("colormap/color_map_hexagon.png", cv2.IMREAD_COLOR)

    # Read from video
    '''
    ret, frame = cap.read()
    if not ret:
        print("Can't read frame (stream end?). Exiting ...")
        break
    '''


    dim = (800, 600)
    resizedframe = cv2.resize(frame, dim)

    # converting to LAB color space
    lab_frame = cv2.cvtColor(resizedframe,cv2.COLOR_BGR2Lab)

    # get info from track bar and apply to LAB threshold filter
    l_min = cv2.getTrackbarPos('lMin','LAB_thresh_tuning')
    l_max = cv2.getTrackbarPos('lMax','LAB_thresh_tuning')
    a_min = cv2.getTrackbarPos('aMin','LAB_thresh_tuning')
    a_max = cv2.getTrackbarPos('aMax','LAB_thresh_tuning')
    b_min = cv2.getTrackbarPos('bMin','LAB_thresh_tuning')
    b_max = cv2.getTrackbarPos('bMax','LAB_thresh_tuning')

    # Masking algorithm
    lb = np.array([l_min, a_min, b_min])
    ub = np.array([l_max, a_max, b_max])

    mask = cv2.inRange(lab_frame, lb, ub)
    masked = cv2.bitwise_and(resizedframe, resizedframe, mask = mask)

    cv2.imshow('LAB thresholds tuning', masked)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()