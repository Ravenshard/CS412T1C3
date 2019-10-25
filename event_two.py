import signal
import rospy
import smach
import smach_ros
import math
import time
import cv2
from math import tanh
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
import detect_shape
from kobuki_msgs.msg import Sound

global shutdown_requested
global checked
global previous_shape


class RotateLeft(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done2', 'follow', 'success2'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        global checked
        while not shutdown_requested:

            target_heading = (self.callbacks.heading + 90) % 360

            turning = True
            previous_difference = None
            while turning:
                if shutdown_requested:
                    return 'done2'
                difference = minimum_angle_between_headings(target_heading, self.callbacks.heading)

                if previous_difference is None:
                    self.twist.angular.z = 0.4
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 1:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = 0.4
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference

            if checked:
                return 'success2'
            else:
                return 'follow'
        return 'done2'


class Follow(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done2', 'stop'])
        self.callbacks = callbacks
        self.prev_error = None
        self.Kp = 1.0 / 50.0
        self.Ki = 1.0 / 50.0
        self.Kd = 1.0 / 50.0
        self.speed = 0.8
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        while not shutdown_requested:
            if self.callbacks.white_mask is not None and self.callbacks.red_mask is not None:

                bottom_white_mask = self.callbacks.white_mask.copy()
                bottom_red_mask = self.callbacks.red_mask.copy()

                # Check if a long red strip has been detected
                h = self.callbacks.h
                w = self.callbacks.w
                search_top = 3 * h / 4
                search_bot = h
                bottom_white_mask[0:search_top, 0:w] = 0
                bottom_white_mask[search_bot:h, 0:w] = 0
                bottom_red_mask[0:search_top, 0:w] = 0
                bottom_red_mask[search_bot:h, 0:w] = 0
                red_pixel_count = cv2.sumElems(bottom_red_mask)[0] / 255
                white_pixel_count = cv2.sumElems(bottom_white_mask)[0] / 255

                if white_pixel_count < 10:
                    print("No white found")
                    print(white_pixel_count)
                    return 'stop'

                RM = cv2.moments(bottom_red_mask)
                if RM['m00'] > 0:
                    ry = int(RM['m01'] / RM['m00'])

                    if red_pixel_count > 500 and ry > 430:
                        print(red_pixel_count)
                        print(ry)
                        print("red found")
                        return 'stop'

                # If there is no significant red line, follow white line
                WM = cv2.moments(bottom_white_mask)

                if WM['m00'] > 0:
                    cx = int(WM['m10'] / WM['m00'])
                    cy = int(WM['m01'] / WM['m00'])

                    # BEGIN CONTROL
                    if self.prev_error is None:
                        error = cx - self.callbacks.w / 2
                        rotation = -(self.Kp * float(error))
                        self.prev_error = error
                    else:
                        error = cx - self.callbacks.w / 2
                        rotation = -(self.Kp * float(error) + self.Kd * (error - self.prev_error))
                        self.prev_error = error
                    self.twist.linear.x = self.speed
                    self.twist.angular.z = rotation
                    self.cmd_vel_pub.publish(self.twist)
                    # END CONTROL
        return 'done2'


class Stop(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done2', 'check', 'rotate_left'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.prev_error = None
        self.Kp = 1.0 / 50.0
        self.Ki = 1.0 / 50.0
        self.Kd = 1.0 / 50.0
        self.speed = 0.8

    def execute(self, userdata):
        global shutdown_requested
        global checked

        if not checked:
            distance = 0.0001
        else:
            distance = 0.35

        while self.callbacks.pose is None:
            time.sleep(1)

        sp = self.callbacks.pose
        ep = sp

        start = time.time()

        while math.sqrt((sp.x - ep.x) ** 2 + (sp.y - ep.y) ** 2) < distance:
            #print(str(math.sqrt((sp.x - ep.x) ** 2 + (sp.y - ep.y) ** 2)) + " "+str(distance))
            if shutdown_requested:
                return 'done2'
            h = self.callbacks.h
            w = self.callbacks.w
            search_top = 3 * h / 4
            search_bot = h
            bottom_white_mask = self.callbacks.white_mask.copy()
            bottom_white_mask[0:search_top, 0:w] = 0
            bottom_white_mask[search_bot:h, 0:w] = 0

            M = cv2.moments(bottom_white_mask)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # BEGIN CONTROL
                if self.prev_error is None:
                    error = cx - self.callbacks.w / 2
                    rotation = -(self.Kp * float(error))
                    self.prev_error = error
                else:
                    error = cx - self.callbacks.w / 2
                    rotation = -(self.Kp * float(error) + self.Kd * (error - self.prev_error))
                    self.prev_error = error
                print(self.speed)
                self.twist.linear.x = self.speed
                self.twist.angular.z = rotation
                self.cmd_vel_pub.publish(self.twist)
                # END CONTROL
                ep = self.callbacks.pose
            else:
                self.twist.linear.x = 2.0
                self.cmd_vel_pub.publish(self.twist)
                if time.time() - start > 0.7:
                    print("break")
                    break

        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_vel_pub.publish(self.twist)

        if checked:
            return 'rotate_left'
        else:
            return 'check'


class Check(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done2', 'rotate_180'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.sound_pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        global checked
        global previous_shape
        while not shutdown_requested:
            symbol_red_mask = self.callbacks.symbol_red_mask.copy()
            symbol_green_mask = self.callbacks.symbol_green_mask.copy()
            h = self.callbacks.h
            w = self.callbacks.w
            symbol_red_mask[0:h / 4, 0:w] = 0
            symbol_red_mask[3 * h / 4:h, 0:w] = 0
            symbol_green_mask[0:h / 4, 0:w] = 0
            symbol_green_mask[3 * h / 4:h, 0:w] = 0

            count = 0
            real_count = 0
            for i in range(10):
                count += detect_shape.count_objects(symbol_red_mask)
            real_count += math.ceil(count / 10)

            count = 0
            for i in range(10):
                count += detect_shape.count_objects(symbol_green_mask)
            real_count += math.ceil(count / 10)

            print(real_count)
            for i in range(int(real_count)):
                self.sound_pub.publish(1)
                time.sleep(1)
            checked = True

            symbol_green_mask = self.callbacks.symbol_green_mask.copy()
            symbol_green_mask[0:h / 4, 0:w] = 0
            symbol_green_mask[3 * h / 4:h, 0:w] = 0

            shapes = detect_shape.detect_shape(symbol_green_mask)[0]
            if len(shapes) > 0:
                previous_shape = shapes[0].value
                print(previous_shape)
            return 'rotate_180'
            '''
            while True:
                shapes = detect_shape.detect_shape(symbol_green_mask)[0]
                if len(shapes) <= 0:
                    continue
                print(shapes)
                for shape in shapes:
                    previous_shape = shape
                    if previous_shape.value != -1:
                        break
            print(previous_shape)
            return 'rotate_180'
            '''
        return 'done1'


class Rotate180(smach.State):

    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['done2', 'follow'])
        self.callbacks = callbacks
        self.twist = Twist()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested
        while not shutdown_requested:

            target_heading = (self.callbacks.heading + 180) % 360

            turning = True
            previous_difference = None
            while turning:
                if shutdown_requested:
                    return 'done2'
                difference = minimum_angle_between_headings(target_heading, self.callbacks.heading)

                if previous_difference is None:
                    self.twist.angular.z = 0.4
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    if difference < 1:
                        turning = False
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = 0.4
                        self.cmd_vel_pub.publish(self.twist)

                if previous_difference != difference:
                    previous_difference = difference

            return 'follow'
        return 'done2'


def minimum_angle_between_headings(a, b):
    heading_difference = a - b
    if heading_difference < 0:
        heading_difference += 360
    if heading_difference > 180:
        heading_difference = b - a
        if heading_difference < 0:
            heading_difference += 360
    return heading_difference


def get_state_machine(callbacks):
    global checked
    checked = False
    sm_event_2 = smach.StateMachine(outcomes=['DONE2', 'SUCCESS2'])
    with sm_event_2:
        smach.StateMachine.add('ROTATE_LEFT', RotateLeft(callbacks),
                               transitions={'done2': 'DONE2', 'follow': 'FOLLOW', 'success2': 'SUCCESS2'})
        smach.StateMachine.add('FOLLOW', Follow(callbacks),
                               transitions={'done2': 'DONE2', 'stop': 'STOP'})
        smach.StateMachine.add('STOP', Stop(callbacks),
                               transitions={'done2': 'DONE2', 'check': 'CHECK', 'rotate_left': 'ROTATE_LEFT'})
        smach.StateMachine.add('CHECK', Check(callbacks),
                               transitions={'done2': 'DONE2', 'rotate_180': 'ROTATE_180'})
        smach.StateMachine.add('ROTATE_180', Rotate180(callbacks),
                               transitions={'done2': 'DONE2', 'follow': 'FOLLOW'})
    return sm_event_2
