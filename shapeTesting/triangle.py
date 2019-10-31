#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import cv2
import cv_bridge
import numpy
import time
import math
from sensor_msgs.msg import Image
from enum import Enum

symbol_green_mask_orig = None
symbol_green_mask_good = [['a']]
bridge = cv_bridge.CvBridge()
shutdown_requested = False
h, w, d = 0, 0, 0

# Square is defined as having 4 verticles over 7000 times out of 10000
# Circle is defined as having -1 verticles over 9000 times out of 10000
# Triangle is defined as having 9 verticles over 9000 times out of 10000

class Shapes(Enum):
    unknown = -1
    triangle = 3
    square = 4
    pentagon = 5
    circle = 9

def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def detect_shape(mask, canvas=None, threshold=100):
    """Detect a shape contained in an image.

    Adappted from: https://stackoverflow.com/questions/11424002/how-to-detect-simple-geometric-shapes-using-opencv
    """
    detected_shapes = []
    moments = []
    _, contours, _ = cv2.findContours(mask, 1, 2)
    for cnt in contours:
        print(cnt)
        rospy.sleep(1)
        if cv2.moments(cnt)["m00"] > threshold:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)

            if len(approx) == 3:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 255, 0), -1)
                detected_shapes.append(Shapes.triangle)
                moments.append(cv2.moments(cnt))

            elif len(approx) == 4:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 0, 255), -1)
                detected_shapes.append(Shapes.square)
                moments.append(cv2.moments(cnt))
            elif len(approx) == 5:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, 255, -1)
                detected_shapes.append(Shapes.pentagon)
                moments.append(cv2.moments(cnt))
            elif len(approx) > 9:
                if canvas != None:
                    cv2.drawContours(canvas, [cnt], 0, (0, 255, 255), -1)
                detected_shapes.append(Shapes.circle)
                moments.append(cv2.moments(cnt))
            else:
                detected_shapes.append(Shapes.unknown)
                moments.append(cv2.moments(cnt))
    return detected_shapes, moments

def stuff():
    global symbol_green_mask_orig, symbol_green_mask_good, h, w, d
    rospy.init_node('triangle')

    image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, image_callback)


    endDict = dict()
    endDict[-1] = 0
    endDict[3] = 0
    endDict[4] = 0
    endDict[5] = 0
    endDict[9] = 0
    count = 0
    while not shutdown_requested and count < 1000:
        if symbol_green_mask_good[0][0] == 'a': continue
        symbol_green_mask = symbol_green_mask_good.copy()

        symbol_green_mask[0:h / 4, 0:w] = 0
        symbol_green_mask[3 * h / 4:h, 0:w] = 0
        shapes = detect_shape(symbol_green_mask)[0]
        if len(shapes) > 0:
            endDict[shapes[0].value] += 1
            previous_shape = shapes[0].value
            # print(previous_shape)
        count += 1
    Circle = {-1:13, 3:0, 4:0, 5:0, 9:987}
    Square = {-1:16, 3:0, 4:853, 5:131, 9:0}
    Triangle = {-1:347, 3:97, 4:68, 5:60, 9:428}
    squareScore = 0
    triangleScore = 0
    circleScore = 0
    # print("endDict")
    print(endDict)
    for key, val in endDict.items():
        # print("LOOP: key: {} \t val: {}".format(key, val))
        # print("AVGS: square: {} \t triangle: {} \t circle: {}".format(Square[key], Triangle[key], Circle[key]))
        squareScore += math.sqrt(pow(Square[key] - val,2))
        triangleScore += math.sqrt(pow(Triangle[key] - val,2))
        circleScore += math.sqrt(pow((Circle[key] - val),2))
        # print("SCORES: squareScore: {} \t triangleScore: {} \t circleScore: {}".format(squareScore, triangleScore, circleScore))
        # rospy.sleep(1)
    # print("FINAL")
    print("circle: {} \t triangle: {} \t square: {}".format(circleScore, triangleScore, squareScore))
    return 'rotate_180'



def image_callback(msg):
    global bridge, symbol_green_mask_good, symbol_green_mask_orig, h, w, d
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape

    upper_green = numpy.array([136, 255, 255])
    lower_green = numpy.array([56, 43, 90])
    symbol_green_mask_orig = cv2.inRange(hsv, lower_green, upper_green)
    blur = cv2.GaussianBlur(symbol_green_mask_orig,(5,5),0)

    symbol_green_mask_good = blur
    cv2.imshow("green window", symbol_green_mask_good)
    cv2.waitKey(3)
    return

if __name__ == '__main__':
    stuff()
