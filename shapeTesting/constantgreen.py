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

def request_shutdown(sig, frame):
    global shutdown_requested
    shutdown_requested = True

def main():
    global symbol_green_mask_orig, symbol_green_mask_good, h, w, d
    rospy.init_node('green')
    image_sub = rospy.Subscriber('/usb_cam/image_raw',
                                  Image, image_callback)
    while not shutdown_requested:
        rospy.sleep(1)
    return

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
    main()
