#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import math 
import threading
import rospy
import copy

class Encoder(object):

    def __init__(self, encoder_resolution, gpio_channels):
        self.A = gpio_channels[0] # [channel1, channel2]
        self.B = gpio_channels[1]
        self.resolution = encoder_resolution

        # Encoder state
        self.tick_count = 0
        self.prev_tick_count = 0
        self.state = 0
        self.prev_time = rospy.Time.now()
        self.angular_velocity = 0
        self.angular_position = 0
        
        # setup GPIO for interrupt
        GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
        GPIO.setwarnings(False)   # Remove warning message
        GPIO.setup(self.A, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup left_pins[0] as input port for left channel 1, 
                                                                            # and we use pull-up resister
        GPIO.setup(self.B, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup left_pins[1] as input port for left channel 2,
                                                                            # and we use pull-up resister
        if GPIO.input(self.A):
            self.state |= 1
        if GPIO.input(self.B):
            self.state |= 2
                                                                            
        GPIO.add_event_detect(self.A, GPIO.BOTH, callback=self.__updateA)  # add detection on left_pins[0] for left channel 1
        GPIO.add_event_detect(self.B, GPIO.BOTH, callback=self.__updateB)  # add detection on left_pins[1] for left channel 2
        
    def __updateA(self, channel):
        state = self.state & 3
        if GPIO.input(self.A):
            state |= 4
        if GPIO.input(self.B):
            state |= 8

        self.state = state >> 2

        if state == 1 or state == 7 or state == 8 or state == 14:
            self.tick_count += 1
        elif state == 2 or state == 4 or state == 11 or state == 13:
            self.tick_count -= 1

            
    def __updateB(self, channel):
        state = self.state & 3
        if GPIO.input(self.A):
            state |= 4
        if GPIO.input(self.B):
            state |= 8

        self.state = state >> 2

        if state == 1 or state == 7 or state == 8 or state == 14:
            self.tick_count += 1
        elif state == 2 or state == 4 or state == 11 or state == 13:
            self.tick_count -= 1


    def reset(self):
        self.tick_count = 0
        self.prev_tick_count = 0
        self.prev_time = rospy.Time.now()
        self.angular_velocity = 0
        self.angular_position = 0


    def __exit__(self, *exc):
        GPIO.cleanup()  # cleanup all GPIOs


    def ticks_to_angle(self, ticks):
        angle = ticks * (2.0*math.pi / self.resolution)
        return angle

    def get_joint_state(self):
        current_tick_count = copy.copy(self.tick_count)
        current_time = rospy.Time.now()
        dt = current_time - self.prev_time
        dt_sec = dt.to_sec()

        delta_ticks = current_tick_count - self.prev_tick_count
        delta_angle = self.ticks_to_angle(delta_ticks)

        self.angular_position += delta_angle
        self.angular_velocity = delta_angle / dt_sec

        self.prev_time = current_time
        self.prev_tick_count = current_tick_count

        return self.angular_position, self.angular_velocity

    def get_angular_velocity(self):
        return self.angular_velocity

    def get_angular_position(self):
        return self.angular_position

    def get_tick_count(self):
        return self.tick_count
