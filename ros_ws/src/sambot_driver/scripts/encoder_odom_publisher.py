#!/usr/bin/env python

# This node will create multi-thread for left and right wheel of the vehicle
# The updated displacement and the velocity of the vehicle will be published
import RPi.GPIO as GPIO
import math
import time
import threading
import rospy
import tf2_ros
import tf_conversions # for transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped

class OdometryPublisher(object):
    def __init__(self,
        left_encoder_channels,
        right_encoder_channels,
        wheel_radius_cm,
        base_width_cm,
        robot_name):
        '''
        - Class constructor
        - Return none
        - Define the hardware information of the vehicle like what parameter
          named, left_encoder_channels and right_encoder_channels are the pin number
          on the jetson nano. Both left and right wheels have two hall-effect sensors 
          each to mimic the rotary encoder (http://robotoid.com/appnotes/circuits-quad-encoding.html)
          function. Assume two wheels are identical, using wheel_radius_cm to represent 
          the radius of two wheels in cm, it will be converted to m before assigning to 
          the self.one_tick_distance_m variable the 'base_width' is the distance
          between the wheel to the center of the vehicle
        - Create some private variables to store specific value for
          functions call
        '''
        self.left_pins = left_encoder_channels # [channel1, channel2], the order of the sensor from left to right in top view
                                               # encoder sensor is on the top half of the motor kit
        self.right_pins = right_encoder_channels # [channel1, channel2]
        self.wheel_radius_m = wheel_radius_cm / 100.0 # radius in 3.3 cm, conver it to m
        self.base_width_m = base_width_cm / 100.0  # 6.0 cm, convert it to m
        self.robot_name = robot_name
        '''
        - Our encoder kit(ROB-12629: http://cdn.sparkfun.com/datasheets/Robotics/multi-chassis%20encoder001.pdf) 
          has 8 ticks, for more information please check datasheet
        - Return the length of one tick in m, precision to 5 after
          the decimal point
        '''
        self.one_tick_distance_m = round((2 * math.pi * self.wheel_radius_m / 8), 5) # calculate one tick distance, 
                                                                                     # total 8 ticks in one revelution
                                                                                              
        # Threading object
        self.stop_threads = True
        self.list_thread = []

        # locker for thread safe
        self.lock = threading.Lock()

        self.calculation_period = 0.3
        self.prev_odom_calc_time = 0.0  # save last time to enter the calc_odom function

        self.tick_counter = [0, 0]  # [left encoder, right encoder] save tick number for calc_odom, 
                                    # will be reset after the calculation

        self.odom_pose = [0.0, 0.0, 0.0]  # x, y, theta (m, m, rad)
        self.odom_vel = [0.0, 0.0, 0.0]  # x, y, theta (m/s, m/s, rad/s)
        self.pub_topic = "/" + self.robot_name + "/odom"  # Topic to publish on
        self.odom_pub = rospy.Publisher(self.pub_topic,
                                        Odometry,
                                        queue_size=50)

        # setup GPIO for interrupt
        GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
        GPIO.setwarnings(False)   # Remove warning message
        GPIO.setup(self.left_pins[0], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup left_pins[0] as input port for left channel 1, 
                                                                          # and we use pull-up resister
        GPIO.setup(self.left_pins[1], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup left_pins[1] as input port for left channel 2,
                                                                          # and we use pull-up resister
        GPIO.add_event_detect(self.left_pins[0], GPIO.BOTH)  # add detection on left_pins[0] for left channel 1
        GPIO.add_event_detect(self.left_pins[1], GPIO.BOTH)  # add detection on left_pins[1] for left channel 2

        GPIO.setup(self.right_pins[0], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup right_pins[0] as input port for right channel 1, 
                                                                           # and we use pull-up resister
        GPIO.setup(self.right_pins[1], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup right_pins[1] as input port for right channel 2, 
                                                                           # and we use pull-up resister
        GPIO.add_event_detect(self.right_pins[0], GPIO.BOTH)  # add detection on right_pins[0] for right channel 1
        GPIO.add_event_detect(self.right_pins[1], GPIO.BOTH)  # add detection on right_pins[1] for right channel 2

    def setup_threads(self):
        '''
        - Setup threads object for left and right encoder interrupts
          and odom calculation

        - Starts the threading object here by calling thread.start()
        '''
        self.stop_threads = False
        thread_left_encoder = threading.Thread(
            target=self.callback_left_encoder
        )

        # daemon must be set before the thread calls
        thread_left_encoder.daemon = False
        thread_right_encoder = threading.Thread(
            target=self.callback_right_encoder
        )
        thread_right_encoder.daemon = False

        thread_odom = threading.Thread(
            target=self.callback_odometry_calculation
        )
        thread_odom.daemon = False

        self.list_thread = [
            thread_left_encoder,
            thread_right_encoder,
            thread_odom]
        for thread in self.list_thread:
            thread.start()
    
        print("Encoder calculation starts!")

    def __exit__(self, *exc):
        '''
        - Assign the stop_threads bool value to True to stop the threading proess
        - Continue following function or other execution after this function call
        '''
        self.stop_threads = True
        GPIO.cleanup()  # cleanup all GPIOs
        for thread in self.list_thread:
            thread.join()
        print("Threads terminated")

    def is_clockwise(self, prev_one, prev_two, curr_one, curr_two):
        '''
        cw:       ccw:                          
        0 1       0 1   
        0 0       1 1
        1 0       1 0
        1 1       0 0
        
        total 8 ticks on our encoder kit ROB-12629. To get direction, we need
        at least two ticks (one for first sensor, one for second sensor). 
        Comparing the sequences of ticks (prev --> current), we can know 
        the direction of the rotation
        
        To achieve this, we must use GPIO.BOTH for interrupt to check 
        both falling and rising edge
        
        return status:
        1: cw, 0: ccw, 2: error
        '''

        if prev_one == 0:
            if prev_two == 0:
                if curr_one == 1 and curr_two == 0: # 0 0 -> 1 0
                    return 1
                elif curr_one == 0 and curr_two == 1: # 0 0 -> 0 1
                    return 0
                else: 
                    return 2
            else: #prev_two is 1
                if curr_one == 0 and curr_two == 0: # 0 1 -> 0 0
                    return 1
                elif curr_one == 1 and curr_two == 1: # 0 1 -> 1 1
                    return 0
                else: 
                    return 2

        else: #prev_one is 1
            if prev_two == 0:
                if curr_one == 1 and curr_two == 1: # 1 0 -> 1 1
                    return 1
                elif curr_one == 0 and curr_two == 0: # 1 0 -> 0 0
                    return 0
                else: 
                    return 2

            else: #prev_two is 1
                if curr_one == 0 and curr_two == 1: # 1 1 -> 0 1
                    return 1
                elif curr_one == 1 and curr_two == 0: # 1 1 -> 1 0
                    return 0
                else: 
                    return 2

    def callback_left_encoder(self):
        '''
        - This is the thread for the left encoder to check same tick to pass two channels detected by GPIO interrupt
          for both edges
        - Use a while loop to detect interrupt for both channels, and compare two states of the sensor reading. The 
          order and the pair of the two sensors' reading are matter to determine the direction of the rotation, 
          more details see is_clockwise function. The return value of the is_clockwise will give us information on the 
          direction of rotation. Increment and decrement of the tick count are different for the left and right wheel, see notes 
          for self.tick_counter changes below.
        '''
        prev_first_sensor_status = 0  # local variable to save previous voltage level reading for channel 1
        prev_second_sensor_status = 0 # local variable to save previous voltage level reading for channel 2
        two_ticks_counter = 0  # used to check both channels are passed by the same tick, 
                               # to get cw or ccw, two interrpts are minimum
        while not self.stop_threads:
            if GPIO.event_detected(self.left_pins[0]) or GPIO.event_detected(self.left_pins[1]):  # update left encoder
                with self.lock: # lock this thread
                    first_sensor = GPIO.input(self.left_pins[0]) #saved first sensor reading status
                    second_sensor = GPIO.input(self.left_pins[1]) #saved second sensor reading status
                    if two_ticks_counter == 0: # if the two_ticks_counter is 0,save the previous status
                        prev_first_sensor_status = first_sensor
                        prev_second_sensor_status = second_sensor
                    else: # two_ticks_counter is 1, compare prev and current sensor's status reading to get direction
                        rotation_status = self.is_clockwise(prev_first_sensor_status, prev_second_sensor_status, first_sensor, second_sensor)
                        if rotation_status == 1: # left cw is decrement
                            self.tick_counter[0] -= 1
                        elif rotation_status == 0: # left ccw is increment
                            self.tick_counter[0] += 1
                    
                    two_ticks_counter += 1  # ticker add one
            
                    if two_ticks_counter >= 2: # reset the ticker if exceed 1
                        two_ticks_counter = 0
                        
                    
    def callback_right_encoder(self):
        '''
        - This is the thread for the right encoder to check same tick to pass two channels detected by GPIO interrupt
          for both edges
        - Use a while loop to detect interrupt for both channels, and compare two states of the sensor reading. The 
          order and the pair of the two sensors' reading are matter to determine the direction of the rotation, 
          more details see is_clockwise function. The return value of the is_clockwise will give us information on the 
          direction of rotation. Increment and decrement of the tick count are different for the left and right wheel, see notes 
          for self.tick_counter changes below.
        '''
        prev_first_sensor_status = 0  # local variable to save previous voltage level reading for channel 1
        prev_second_sensor_status = 0 # local variable to save previous voltage level reading for channel 2
        two_ticks_counter = 0  # used to check both channels are passed by the same tick, 
                                    # to get cw or ccw, two interrpts are minimum
        while not self.stop_threads:
            if GPIO.event_detected(self.right_pins[0]) or GPIO.event_detected(self.right_pins[1]):  # update right encoder       
                with self.lock:  # lock this thread
                    first_sensor = GPIO.input(self.right_pins[0]) #saved first sensor reading status
                    second_sensor = GPIO.input(self.right_pins[1]) #saved second sensor reading status
                    if two_ticks_counter == 0: # if the two_ticks_counter is 0,save the previous status
                        prev_first_sensor_status = first_sensor
                        prev_second_sensor_status = second_sensor
                    else: # two_ticks_counter is 1, compare prev and current sensor's status reading to get direction
                        rotation_status = self.is_clockwise(prev_first_sensor_status, prev_second_sensor_status, first_sensor, second_sensor)
                        if rotation_status == 1: # right cw is increment
                            self.tick_counter[1] += 1
                        elif rotation_status == 0: # right ccw is decrement
                            self.tick_counter[1] -= 1
                    
                    two_ticks_counter += 1  # ticker add one
            
                    if two_ticks_counter >= 2: # reset the ticker if exceed 1
                        two_ticks_counter = 0

    def calc_and_publish_odometry(self):
        '''
        - This function update the displacement and velocity of the vehicle
          within a certain time (use time_interval, not self.calculation_period 
          is not accuracy enough)
        - The calculation will be ignored only if two wheels are stopped
        - If return False, the velocity of two wheel will be set to 0.0
        '''
        print('ticks: ' + str(self.tick_counter[0]) + ', ' + str(self.tick_counter[1]))
        return True

    def publish_odom(self, odom_publish_timestamp):
        '''
        - A ROS publisher to publish the tf2 and odometry information to the master
        - Reference code for tf2 : http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29
        '''
        print('pose: ' + str(self.odom_pose[0]) + ', ' + str(self.odom_pose[1]))
        odom_broadcaster = tf2_ros.TransformBroadcaster()
        transform = TransformStamped()
         # first, we'll publish the transform over tf2
        transform.header.stamp = odom_publish_timestamp
        transform.header.frame_id = self.robot_name + "/odom"
        transform.child_frame_id = self.robot_name + "/chassis"

        transform.transform.translation.x = self.odom_pose[0]
        transform.transform.translation.y = self.odom_pose[1]
        transform.transform.translation.z = 0.0

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.odom_pose[2])
        transform.transform.rotation.x = odom_quat[0]
        transform.transform.rotation.y = odom_quat[1]
        transform.transform.rotation.z = odom_quat[2]
        transform.transform.rotation.w = odom_quat[3]
        odom_broadcaster.sendTransform(transform)

        # next, we'll publish the odometry message over ROS
        odom_msg = Odometry()
        odom_msg.header.stamp = odom_publish_timestamp
        odom_msg.header.frame_id = self.robot_name + "/odom"

        # set the position
        odom_msg.pose.pose = Pose(Point(self.odom_pose[0], self.odom_pose[1], 0.0), Quaternion(*odom_quat))

        # set the velocity
        odom_msg.child_frame_id = self.robot_name + "/chassis"
        odom_msg.twist.twist = Twist(Vector3(self.odom_vel[0], self.odom_vel[1], 0.0), Vector3(0.0, 0.0, self.odom_vel[2]))

        # publish the message
        self.odom_pub.publish(odom_msg)

    def callback_odometry_calculation(self):
        '''
        - A thread to keep updating the odometry then publish updated odometry information after it.
        - Sleep in some period depends on self.calculation_period
        '''
        self.prev_odom_calc_time = time.time()  # it should initialize to the first time enter this function
                                                           # would only define once
        while not self.stop_threads:
            rospy.sleep(self.calculation_period)
            with self.lock:
                self.calc_and_publish_odometry()

if __name__ == '__main__':
    rospy.init_node('odometry_publisher')
    robot_name = rospy.get_param('~robot_name', 'sambot1')
    wheel_radius = rospy.get_param('~wheel_radius', 3.3)
    wheel_base = rospy.get_param('~wheel_base', 6.25)
    encoder_publisher = OdometryPublisher([35, 37], [38, 40], wheel_radius, wheel_base, robot_name)  # GPIO pin 35 and 37 is used for left encoder sensors
                                                                                                     # Also from the top view of the hardware assembly 
                                                                                                     # (place sensor to the top half of the motor, not the downside), 
                                                                                                     # 35 is the first channel and 37 is second channel. Same for 
                                                                                                     # the pin 38 and 40, these two are for right encoder. 3.3 is wheel
                                                                                                     # radius in cm, while 6.25 cm is the distance between wheel to 
    encoder_publisher.setup_threads()

