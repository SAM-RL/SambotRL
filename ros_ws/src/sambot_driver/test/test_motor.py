#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from motor.motor import Motor

i2c_bus = 1
motor_channel = 1
motor_alpha = 1

value = 0.0
offset = 0.1

if __name__ == '__main__':
    motor_driver = Adafruit_MotorHAT(i2c_bus=i2c_bus)
    motor_left = Motor(motor_driver, channel=1, alpha=1)
    motor_right = Motor(motor_driver, channel=2, alpha=1)
    motor_left.value = 0.0 
    motor_right.value = 0.0

    motor_left.value = 0.55
    motor_right.value = 0.615

    while True:
        print(f"{motor_left.value}, {motor_right.value}")
        time.sleep(2)
        #motor_left.value += 0.01
                                                
