#!/usr/bin/env python

import signal
import rospy
import smach
import smach_ros
import math
from geometry_msgs.msg import Twist
import numpy as np
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from kobuki_msgs.msg import Led


class Box1(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'box3', 'box4',
                                             'box5', 'box6', 'box7', 'box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.86572667681
        self.target.target_pose.pose.position.y = -1.23889378088
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 1:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off
        # TODO detect shape

        if shutdown_requested:
            return 'done4'
        else:
            return 'box2'


class Box2(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box1', 'box3', 'box4',
                                             'box5', 'box6', 'box7', 'box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -2.48666366809
        self.target.target_pose.pose.position.y = -1.87504164082
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 2:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box3'


class Box3(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'box1', 'box4',
                                             'box5', 'box6', 'box7', 'box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.81187055824
        self.target.target_pose.pose.position.y = -2.37934982584
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 3:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box4'


class Box4(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'box3', 'box1',
                                             'box5', 'box6', 'box7', 'box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.617115325015
        self.target.target_pose.pose.position.y = -1.74971739038
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 4:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box5'


class Box5(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'box3', 'box4',
                                             'box1', 'box6', 'box7', 'box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.792065221849
        self.target.target_pose.pose.position.y = -3.43328318514
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 5:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box6'


class Box6(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'box3', 'box4',
                                             'box5', 'box1', 'box7', 'box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.45466633388
        self.target.target_pose.pose.position.y = -3.91585715022
        #self.target.target_pose.pose.position.x = -0.344961246087
        #self.target.target_pose.pose.position.y = -4.06018585631
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 6:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box7'


class Box7(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'box3', 'box4',
                                             'box5', 'box6', 'box1', 'box8', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -0.110382720208
        self.target.target_pose.pose.position.y = -2.42328070669
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 7:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off

        if shutdown_requested:
            return 'done4'
        else:
            return 'box8'


class Box8(smach.State):
    def __init__(self, callbacks):
        smach.State.__init__(self, outcomes=['box2', 'box3', 'box4',
                                             'box5', 'box6', 'box7', 'box1', 'done4'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #self.client.wait_for_server()

        self.target = MoveBaseGoal()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.pose.position.x = -1.3672780018
        self.target.target_pose.pose.position.y = -2.99154837699
        self.target.target_pose.pose.orientation.x = 0.0
        self.target.target_pose.pose.orientation.y = 0.0
        self.target.target_pose.pose.orientation.z = -0.372922294621
        self.target.target_pose.pose.orientation.w = 0.927862577203

        self.callbacks = callbacks
        self.led_pub = rospy.Publisher('/mobile_base/commands/led1', Led, queue_size=1)

    def execute(self, userdata):
        global shutdown_requested

        self.client.send_goal(self.target)
        self.client.wait_for_result()
        print("Goal reached")

        if self.callbacks.tag_visible:
            self.led_pub(1)  # green
            time.sleep(5)
            self.led_pub(0)  # off
        elif self.callbacks.target_box == 8:
            self.led_pub(3)  # red
            time.sleep(5)
            self.led_pub(0)  # off

        return 'done4'


def test(callbacks, current_box):
    if callbacks.tag_visible:
        # Callbacks
        time.sleep(5)
    elif callbacks.target_box == current_box:

        time.sleep(5)


def get_state_machine(callbacks):

    # Create done outcome which will stop the state machine
    sm_event_4 = smach.StateMachine(outcomes=['DONE4', 'SUCCESS4'])

    with sm_event_4:
        smach.StateMachine.add('BOX1', Box1(callbacks),
                               transitions={'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done4': 'DONE4'})
        smach.StateMachine.add('BOX2', Box2(callbacks),
                               transitions={'box1': 'BOX1',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done4': 'DONE4'})
        smach.StateMachine.add('BOX3', Box3(callbacks),
                               transitions={'box2': 'BOX2',
                                            'box1': 'BOX1', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done4': 'DONE4'})
        smach.StateMachine.add('BOX4', Box4(callbacks),
                               transitions={'box2': 'BOX2',
                                            'box3': 'BOX3', 'box1': 'BOX1',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done4': 'DONE4'})
        smach.StateMachine.add('BOX5', Box5(callbacks),
                               transitions={'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box1': 'BOX1', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done4': 'DONE4'})
        smach.StateMachine.add('BOX6', Box6(callbacks),
                               transitions={'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box1': 'BOX1',
                                            'box7': 'BOX7', 'box8': 'BOX8',
                                            'done4': 'DONE4'})
        smach.StateMachine.add('BOX7', Box7(callbacks),
                               transitions={'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box1': 'BOX1', 'box8': 'BOX8',
                                            'done4': 'DONE4'})
        smach.StateMachine.add('BOX8', Box8(callbacks),
                               transitions={'box2': 'BOX2',
                                            'box3': 'BOX3', 'box4': 'BOX4',
                                            'box5': 'BOX5', 'box6': 'BOX6',
                                            'box7': 'BOX7', 'box1': 'BOX1',
                                            'done4': 'DONE4'})
    return sm_event_4


'''
header: 
  seq: 125
  stamp: 
    secs: 1571336109
    nsecs: 238852016
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: -1.4029582974
      y: -0.161230396952
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.409603401919
      w: 0.912263697149
  covariance: [0.003383795001103751, 0.0030187297400489366, 0.0, 0.0, 0.0, 0.0, 0.0030187297400489366, 0.0106325757067692, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0017897886634780384]
'''