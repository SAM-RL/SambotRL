#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import time
msg = """
Go a straight line in 0.2m/s for 2s,
then turn left for 1.5*0.2 = 0.3 rad/s 
for 0.3s to make a close right angle. 
Repeat previous steps for 4 times to 
draw a square shape. The actual 
performance may differ from the 
expected displacement

The main purpose is to measure the
orientation changes, it should 
return a value closes to 360 deg
"""

moveCommand = {
        'forward':(1,0,0,0),
        'back':   (-1,0,0,0),
        'left':   (0,0,0,1), 
        'right':  (0,0,0,-1),
        'stop':   (0,0,0,0),
    }

speed = 0.4
turn = speed * 1.5
x = 0
y = 0
z = 0
th = 0

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def stop():
    twist = Twist()
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
    pub1.publish(twist)

def forward(duration):
    command =  'forward'  # can use 'back' 
    check_point = time.time()
    x = moveCommand[command][0]
    time_diff = 0
    twist = Twist()
    twist.linear.x = x*speed
    pub1.publish(twist)
    while time_diff < duration:
        time_diff = time.time() - check_point

def left_turn(duration):
    command = 'left'
    check_point = time.time()
    th = moveCommand[command][3]
    time_diff = 0
    twist = Twist()
    twist.angular.z = th*turn
    pub1.publish(twist)
    while time_diff < duration:
        time_diff = time.time() - check_point

if __name__=="__main__":
    pub1 = rospy.Publisher('/cmd_vel/sambot1', Twist, queue_size = 1)
    rospy.init_node('teleop_draw_straight_line')
    print(msg)
    duration_straight = 2
    duration_turn = 0.3
    # time.sleep(5)
    try:
        for i in range (4):
            forward(duration_straight)
            stop()
            time.sleep(0.5)
            left_turn(duration_turn)
            stop()
            time.sleep(0.5)
    except Exception as e:
        print(e)

    finally:
        stop()
        print("\nFinish drawing square!")
