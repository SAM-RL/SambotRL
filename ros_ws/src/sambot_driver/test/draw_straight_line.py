#!/usr/bin/env python

from __future__ import print_function

import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty
import time
msg = """
Go a straight line in 0.4m/s for 2s, the
actual performance may differ from the 
expected displacement

This test file is to verify the accuracy 
of the displacement
"""

moveCommand = {
        'forward':(1,0,0,0),
        'back':   (-1,0,0,0),
        'left':   (0,0,0,1), 
        'right':  (0,0,0,-1),
        'stop':   (0,0,0,0),
    }

speed = 0.4
turn = 0
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

if __name__=="__main__":
    pub1 = rospy.Publisher('/cmd_vel/sambot1', Twist, queue_size = 1)
    rospy.init_node('teleop_draw_straight_line')
    print(msg)
    duration_straight = 2
    # time.sleep(5)
    try:
        forward(duration_straight)
    except Exception as e:
        print(e)

    finally:
        stop()
        print("\nFinish drawing a straight line!")

 
