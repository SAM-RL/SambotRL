#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

class Encoder(object):
    """
    Encoder class allows to work with rotary encoder
    which connected via two pin A and B.
    Works only on interrupts because all RPi pins allow that.
    This library is a simple port of the Arduino Encoder library
    (https://github.com/PaulStoffregen/Encoder) 
    """
    def __init__(self, A, B):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(A, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(B, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.A = A
        self.B = B
        self.pos = 0
        self.state = 0
        if GPIO.input(A):
            self.state |= 1
        if GPIO.input(B):
            self.state |= 2
        GPIO.add_event_detect(A, GPIO.BOTH, callback=self.__updateA)
        GPIO.add_event_detect(B, GPIO.BOTH, callback=self.__updateB)

    """
    update() calling every time when value on A or B pins changes.
    It updates the current position based on previous and current states
    of the rotary encoder.
    """
    def __updateA(self, channel):
        state = self.state & 3
        if GPIO.input(self.A):
            state |= 4
        if GPIO.input(self.B):
            state |= 8

        self.state = state >> 2

        if state == 1 or state == 7 or state == 8 or state == 14:
            self.pos += 1
        elif state == 2 or state == 4 or state == 11 or state == 13:
            self.pos -= 1

            
    def __updateB(self, channel):
        state = self.state & 3
        if GPIO.input(self.A):
            state |= 4
        if GPIO.input(self.B):
            state |= 8

        self.state = state >> 2

        if state == 1 or state == 7 or state == 8 or state == 14:
            self.pos += 1
        elif state == 2 or state == 4 or state == 11 or state == 13:
            self.pos -= 1


    """
    read() simply returns the current position of the rotary encoder.
    """
    def read(self):
        return self.pos


if __name__ == '__main__':
    enc_left = Encoder(35, 37)
    enc_right = Encoder(38, 40)
    while True:
        pos_l = enc_left.read()
        pos_r = enc_right.read()
        print(f"{pos_l}, {pos_r}")
        time.sleep(0.100)
                                                
