#!/usr/bin/env python3
# from field_no_loc_state_9_actions import SpatialTemporalFieldNoLocStateWithGradRewardConcNineActions
# import gym
from gym_field.envs.field_no_loc_state_9_actions import SpatialTemporalFieldNoLocStateWithGradRewardConcNineActions
import gym
import gym_field
import time
import numpy as np
import torch

# from lib import dqn
# from lib import algorithms2
import dqn
import algorithms2

import collections

import rospy
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from sambot_base.msg import MoveTowardGoalGoal, MoveTowardGoalAction
from sambot_msgs.msg import NavState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import math
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

import os

PI = math.pi
CELL_TO_METER = 0.2286
HALF_FIELD_SIZE = 6 * CELL_TO_METER
SIM_CELL_TO_METER = 0.027432
OFFSET = CELL_TO_METER # TODO: adjust this param
STARTING_POS = [75, 75] # TODO: validate this value (currently based on (CELL_TO_METER/SIM_CELL_TO_METER)*2)
HALF_VIEW_SCOPE_SIZE = 5

robot_name = ["sambot1", "sambot2", "sambot3", "sambot4"]
active_robots = [False, False, False, False]
offset = np.array([[OFFSET, -OFFSET], [OFFSET, OFFSET], [-OFFSET, OFFSET], [-OFFSET, -OFFSET]])

DEFAULT_ENV_NAME = "field-no-loc-state-with-grad-reward-9-actions-v0"
FOLDER_PATH = "/home/thinh/workspace/ros_ws/src/sambot_rl/scripts/"
TWO_SRC_MODEL_ADDR = FOLDER_PATH + \
    "dqn_basic_with_grad_9_no_loc_state_2_srcs_grad_reward_random_start_loc_large_network_9_act/field-no-loc-state-with-grad-reward-9-actions-v0-final_best_10169-steps_2000272.dat"
EXPERIMENT_NAME = "dqn_basic_with_grad_9_no_loc_state_2_srcs_grad_reward_random_start_loc_large_network_9_act"
OUTPUT_TESTING_DIR = FOLDER_PATH + "testing_output_2_srcs_full_exp"
NUM_SOURCES = 2
EPISODE_NUM = 1
TESTING_FIELD = None
NUM_EXP_TO_AVG = 10
CUDA = False
MODEL = TWO_SRC_MODEL_ADDR

MIN_DISTANCE = 0.75 * CELL_TO_METER # TODO: adjust this param
TEST_MODE = True

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

def sendGoalToRobot(prev_pos, next_pos, force_move=False):
    prev_pos_in_meters = np.array(prev_pos) * SIM_CELL_TO_METER
    next_pos_in_meters = np.array(next_pos) * SIM_CELL_TO_METER 
    dist = np.sqrt((prev_pos_in_meters[0]-next_pos_in_meters[0])**2 + (prev_pos_in_meters[1]-next_pos_in_meters[1])**2)
    if (force_move or dist >= MIN_DISTANCE):
        next_pos_in_meters = (next_pos_in_meters - HALF_FIELD_SIZE) * np.array([-1,1])
        print(f"dist:{dist} | SEND TO ROBOTS: {next_pos_in_meters}")
        if TEST_MODE:
            # time.sleep(0.1)
            return True
        pos = np.zeros((4, 3))
        for j in range(4):
            pos[j, :2] = next_pos_in_meters[:2] + offset[j]

        for client, active, pos_i in zip(clients, active_robots, pos):
            if active:
                client.send_goal(composeGoal(*pos_i))

        msg_nav = NavState()
        msg_nav.goal.extend(next_pos_in_meters[:2])
        pub_nav_state.publish(msg_nav)

        for active, name, pos_i in zip(active_robots, robot_name, pos):
            if active:
                publishGoalToTF(*pos_i, name, broadcaster)
        success = all(client.wait_for_result(rospy.Duration(30.0)) if active else True for client, active in zip(clients, active_robots))
        # time.sleep(0.5)
        return True
    else:
        return False

def field_to_cvimage(field_arr, size=(800,800)):
    field_arr = ((field_arr / np.max(field_arr)) * 255).astype(np.uint8)
    img = cv2.resize(field_arr, size, interpolation = cv2.INTER_AREA)
    img = cv2.cvtColor(img,cv2.COLOR_GRAY2RGB)
    img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
    return img

if __name__ == "__main__":

    # Setup Action Client
    # rospy.init_node('dqn_experiment_node')
    # rate = rospy.Rate(10)

    # rospy.loginfo(os.getcwd())

    # broadcaster = TransformBroadcaster()
    # pub_nav_state = rospy.Publisher("/center_goal", NavState, queue_size=100)
    # pub_env_field = rospy.Publisher('/env_field_img', Image, queue_size=10)
    # pub_agent_field = rospy.Publisher('/agent_field_img', Image, queue_size=10)

    # clients = [actionlib.SimpleActionClient("/"+name+"/move_toward_goal_server", MoveTowardGoalAction) for name in robot_name]

    # rospy.loginfo("Waiting for action servers to start.")
    # for client, active in zip(clients, active_robots):
    #     if active:
    #         client.wait_for_server()

    # rospy.loginfo("Action servers are ready.")
    
    # Setup DQN
    device = torch.device("cuda:0" if CUDA else "cpu")

    # adv_diff_params = {"vx": -0.6, "vy": 0.8}  # Update this
    adv_diff_params = {"vx": 0.0, "vy": 0.0, "k": 1.8, "dx": 0.8, "dy": 0.8}  # Update this


    env = gym.make(DEFAULT_ENV_NAME, learning_experiment_name=EXPERIMENT_NAME,
                   output_dir=OUTPUT_TESTING_DIR, num_sources=NUM_SOURCES,
                   adv_diff_params=adv_diff_params, testing_field=TESTING_FIELD, view_scope_half_side=HALF_VIEW_SCOPE_SIZE)

    net = dqn.DQN1(env.observation_space.shape, 512,
                   env.action_space.n).to(device)
    state = torch.load(MODEL, map_location=lambda stg, _: stg)
    net.load_state_dict(state)

    # ----------------------------------------------------------------------
    num_sources = NUM_SOURCES - 1

    # Source detector
    source_detector = algorithms2.SourceDetector(conc_threshold=13,
        buffer_size=30, position_buffer_size=50, grad_threshold=0.3)

    # Destination Chooser
    destination_chooser = algorithms2.DestinationChooser(35)

    # Controller
    controller = None

    # Begin in RL mode
    in_rl_mode = True

    # Current destination
    destination = None
    path_to_destination = None

    prev_pos = next_pos = STARTING_POS
    state = env.reset(start_pos=prev_pos)
    total_reward = 0.0
    c = collections.Counter()
    steps = 0

    # br = CvBridge()
    # curr_field_img = field_to_cvimage(env.env_curr_field.T)
    # pub_env_field.publish(br.cv2_to_imgmsg(curr_field_img))
    done=False
    source_found = False

    sendGoalToRobot(prev_pos=prev_pos, next_pos=next_pos, force_move=True)

    while True:
        # print(f"IS DONE -----> [rl:{in_rl_mode},done:{done},found:{source_found}]")
        if in_rl_mode:
            state_v = torch.tensor(np.array([state], copy=False)).to(device)
            q_vals = net(state_v).data.cpu().numpy()[0]
            action = np.argmax(q_vals)
            c[action] += 1
            reward, state, done, observation = env.step(action)
            next_pos = observation['location']
            total_reward += reward
            # rospy.loginfo(f"ACTION: {action}, STATE: {next_pos}, SOURCES: {num_sources}")
            # Check if source detected
            if (num_sources != 0):
                source_found = source_detector.is_source_detected(
                    state, next_pos)
                if (source_found):
                    in_rl_mode = False
                    num_sources -= 1
                    destination, path_to_destination = destination_chooser.find_destination(
                        next_pos, env.agent_field_visited)
                    # env.view_testing_episode_state(
                    #     EPISODE_NUM, steps, path=path_to_destination)
                    steps += 1
                    source_detector.reset()
                    continue
        else:
            if controller == None:
                controller = algorithms2.Controller(next_pos,
                                                    path_to_destination, destination)

            action, destination_reached = controller.choose_action(next_pos)

            if (destination_reached):
                print("Destination Reached, going back to RL mode")
                in_rl_mode = True
                destination = None
                controller = None
                # env.view_testing_episode_state(
                #     EPISODE_NUM, steps, path=path_to_destination)
            else:
                reward, state, done, observation = env.step(action)
                next_pos = observation['location']
                total_reward += reward
        
        if done:
            env.view_testing_episode_state(
                EPISODE_NUM, steps, path=path_to_destination)
            # print(env.agent_curr_field)
            np.savetxt('agent_field.csv', env.agent_curr_field, delimiter=',')
            np.savetxt('env_field.csv', env.env_curr_field, delimiter=',')
            cv2.imwrite(f"field_{steps}.png", field_to_cvimage(env.agent_curr_field.T))
            break
        else:
            if sendGoalToRobot(prev_pos=prev_pos, next_pos=next_pos):
                prev_pos = next_pos

        # curr_field_img = field_to_cvimage(env.env_curr_field.T)
        # agent_field_img = field_to_cvimage(env.agent_curr_field.T, size=(350,350))
        # pub_env_field.publish(br.cv2_to_imgmsg(curr_field_img))
        # pub_agent_field.publish(br.cv2_to_imgmsg(agent_field_img))

        steps += 1

        # rate.sleep()

    # for client in clients:
    #     client.cancel_goal()

    # while not rospy.is_shutdown():
    #     rate.sleep()

    # cv2.destroyAllWindows()

















