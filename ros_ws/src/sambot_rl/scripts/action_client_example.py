#!/usr/bin/env python

import rospy
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from sambot_base.msg import MoveTowardGoalGoal, MoveTowardGoalAction
from sambot_msgs.msg import NavState
from geometry_msgs.msg import TransformStamped
from tf.broadcaster import TransformBroadcaster

import math
import numpy as np

PI = math.pi
CELL_TO_METER = 0.0254 * 9
OFFSET = CELL_TO_METER

TOLERANCE_XY = 0.1
TOLERANCE_THETA = 0.2

WAYPOINTS_MAX = 20
WAYPOINTS_LENGTH = 20
waypoints = np.array([[-4, 4, -PI/2], [-4, 3, -PI/4], [-3, 3, -PI/4], [-2, 2, -PI/4], [-2, 1, -PI/2],
                      [-2, 0, -PI], [-3, 0, -3*PI/4], [-4, -1, -PI/2], [-4, -2, -PI/2], [-4, -3, 0],
                      [-3, -3, PI/4], [-2, -2, PI/4], [-1, -1, PI/4], [0, 0, PI/4], [1, 1, 0],
                      [2, 1, PI/4], [3, 2, PI/2], [3, 3, PI/2], [3, 4, PI/4], [4, 5, -PI/2]])

robot_name = ["sambot1", "sambot2", "sambot3", "sambot4"]
active_robots = [True, True, True, True]

offset = np.array([[OFFSET, -OFFSET], [OFFSET, OFFSET], [-OFFSET, OFFSET], [-OFFSET, -OFFSET]])

def composeGoal(x, y, theta):
    goal = MoveTowardGoalGoal()
    goal.x = x
    goal.y = y
    goal.theta = theta
    return goal

def publishGoalToTF(x, y, theta, name, broadcaster):
    q = tf.transformations.quaternion_from_euler(0, 0, theta)
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "map"
    transform.child_frame_id = name + "/goal"
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]
    broadcaster.sendTransform(transform)

if __name__ == '__main__':
    rospy.init_node('move_toward_goal_client')

    broadcaster = TransformBroadcaster()
    pub_nav_state = rospy.Publisher("/center_goal", NavState, queue_size=100)

    clients = [actionlib.SimpleActionClient("/"+name+"/move_toward_goal_server", MoveTowardGoalAction) for name in robot_name]

    rospy.loginfo("Waiting for action servers to start.")
    for client, active in zip(clients, active_robots):
        if active:
            client.wait_for_server()

    rospy.loginfo("Action servers are ready, sending goal.")
    rate = rospy.Rate(2.0)

    for i in range(WAYPOINTS_MAX):
        pos = np.zeros((4, 3))
        for j in range(4):
            pos[j, :2] = CELL_TO_METER * waypoints[i % WAYPOINTS_LENGTH, :2] + offset[j]
            pos[j, 2] = waypoints[i % WAYPOINTS_LENGTH, 2]

        for client, active, pos_i in zip(clients, active_robots, pos):
            if active:
                client.send_goal(composeGoal(*pos_i))

        msg_nav = NavState()
        msg_nav.goal.extend(CELL_TO_METER * waypoints[i % WAYPOINTS_LENGTH, :2])
        pub_nav_state.publish(msg_nav)

        for active, name, pos_i in zip(active_robots, robot_name, pos):
            if active:
                publishGoalToTF(*pos_i, name, broadcaster)

        success = all(client.wait_for_result(rospy.Duration(30.0)) if active else True for client, active in zip(clients, active_robots))
        if not success:
            break

        rate.sleep()

    while not rospy.is_shutdown():
        rate.sleep()

    for client in clients:
        client.cancel_goal()
