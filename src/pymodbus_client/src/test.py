#!/usr/bin/env python3
import numpy as np
import cv2 as cv


def check_image(image):
    S_min = 67
    V_min = 78

    S_max = 25517
    V_max = 255

    pos_x = [881, 542, 211]
    pos_y = [202, 216, 211]

    result_y = False
    result_r = False
    result_b = False
    missing = False

    height, width = image.shape[:2]
    y=0
    w=300
    h=height

    #YELLOW
    x=700
    croppedImg = image[y:y+h, x:x+w]
    H_min = 17
    H_max = 30
    image_hsv = cv.cvtColor(croppedImg, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    count = cv.countNonZero(image_seg)
    if count > 23000:
        result_y = True

    #RED
    x=400
    croppedImg = image[y:y+h, x:x+w]
    H_min = 125
    H_max = 180
    image_hsv = cv.cvtColor(croppedImg, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    count = cv.countNonZero(image_seg)
    if count > 14000:
        result_r = True


    #BLUE
    x=100
    croppedImg = image[y:y+h, x:x+w]
    H_min = 82
    H_max = 118
    image_hsv = cv.cvtColor(croppedImg, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    count = cv.countNonZero(image_seg)
    if count > 7500:
        result_b = True

    H_min = 0
    S_min = 0
    V_min = 0
    H_max = 180
    S_max = 130
    V_max = 255
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    for i in range(3):
        x = pos_x[i]
        y = pos_y[i]
        if image_seg[y, x] == 255:
            missing = True

    return result_y, result_r, result_b, missing


if __name__ == "__main__":
    image = cv.imread("/home/lasse/Desktop/correctPos.png", cv.IMREAD_COLOR);
    result_y, result_r, result_b, missing = check_image(image)
    print("yellow:", result_y)
    print("red:", result_r)
    print("blue:", result_b)
    print("missing:", missing)
