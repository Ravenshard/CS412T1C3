#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import cv2
import cv_bridge
import numpy
import time
from sensor_msgs.msg import Image
from enum import Enum

symbol_green_mask_orig = None
symbol_green_mask_good = [['a']]
bridge = cv_bridge.CvBridge()
shutdown_requested = False
h, w, d = 0, 0, 0

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

    image_sub = rospy.Subscriber('/usb_cam/image_raw',
                                      Image, image_callback)


    endDict = dict()
    endDict[-1] = 0
    endDict[3] = 0
    endDict[4] = 0
    endDict[5] = 0
    endDict[9] = 0
    print("2")
    rospy.sleep(1)
    print("1")
    rospy.sleep(1)
    count = 0
    while not shutdown_requested and count < 10000:
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
    print(endDict)
    return 'rotate_180'



def image_callback(msg):
    global bridge, symbol_green_mask_good, symbol_green_mask_orig, h, w, d
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    h, w, d = image.shape


    upper_red_a = numpy.array([20, 255, 255])
    lower_red_a = numpy.array([0, 100, 100])
    red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

    upper_red_b = numpy.array([255, 255, 255])
    lower_red_b = numpy.array([150, 100, 100])
    red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)
    red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

    upper_red_a = numpy.array([20, 255, 255])
    lower_red_a = numpy.array([0, 200, 60])
    red_mask_a = cv2.inRange(hsv, lower_red_a, upper_red_a)

    upper_red_b = numpy.array([255, 255, 255])
    lower_red_b = numpy.array([150, 200, 60])
    red_mask_b = cv2.inRange(hsv, lower_red_b, upper_red_b)

    symbol_red_mask = cv2.bitwise_or(red_mask_a, red_mask_b)

    upper_green = numpy.array([136, 255, 255])
    lower_green = numpy.array([56, 43, 90])
    symbol_green_mask_orig = cv2.inRange(hsv, lower_green, upper_green)

    bottom_red_mask = red_mask.copy()
    search_top = 3 * h / 4
    search_bot = h
    bottom_red_mask[0:search_top, 0:w] = 0
    bottom_red_mask[search_bot:h, 0:w] = 0
    red_pixel_count = cv2.sumElems(bottom_red_mask)[0] / 255

    symbol_red_mask2 = symbol_red_mask.copy()
    symbol_red_mask2[0:h/4, 0:w] = 0
    symbol_red_mask2[3*h/4:h, 0:w] = 0

    symbol_green_mask_good = symbol_green_mask_orig
    cv2.imshow("green window", symbol_green_mask_good)
    cv2.waitKey(3)
    return

if __name__ == '__main__':
    stuff()
