#!/usr/bin/env python

#This is the testing file to test total four channels
#on two wheels, each channel should pass 4 times if
#you rotate the wheel in a complete rotation
import RPi.GPIO as GPIO
import threading

class Wheels_testing(object):
    def __init__(self,
        left_encoder_channels,
        right_encoder_channels):
  
        self.left_pins = left_encoder_channels # [channel1, channel2], the order of the sensor from left to right in top view
                                               # encoder sensor is on the top half of the motor kit
        self.right_pins = right_encoder_channels # [channel1, channel2]

        self.left_count = 0
        self.right_count = 0

        # setup GPIO for interrupt
        GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
        GPIO.setwarnings(False)   # Remove warning message
        GPIO.setup(self.left_pins[0], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup left_pins[0] as input port for left channel 1, 
                                                                          # and we use pull-up resister
        GPIO.setup(self.left_pins[1], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup left_pins[1] as input port for left channel 2,
                                                                          # and we use pull-up resister
        GPIO.add_event_detect(self.left_pins[0], GPIO.RISING)  # add detection on left_pins[0] for left channel 1
        GPIO.add_event_detect(self.left_pins[1], GPIO.RISING)  # add detection on left_pins[1] for left channel 2

        GPIO.setup(self.right_pins[0], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup right_pins[0] as input port for right channel 1, 
                                                                           # and we use pull-up resister
        GPIO.setup(self.right_pins[1], GPIO.IN, pull_up_down=GPIO.PUD_UP)  # setup right_pins[1] as input port for right channel 2, 
                                                                           # and we use pull-up resister
        GPIO.add_event_detect(self.right_pins[0], GPIO.RISING)  # add detection on right_pins[0] for right channel 1
        GPIO.add_event_detect(self.right_pins[1], GPIO.RISING)  # add detection on right_pins[1] for right channel 2
        
        self.setup_threads()

    def setup_threads(self):
        '''
        - Setup threads object for left and right encoder interrupts

        - Starts the threading object here by calling thread.start()
        '''
        thread_left_encoder = threading.Thread(
            target=self.left_encoder
        )
       
        thread_left_encoder.daemon = False # daemon must be set before the thread calls

        thread_right_encoder = threading.Thread(
            target=self.right_encoder
        )
        thread_right_encoder.daemon = False

        list_thread = [
            thread_left_encoder,
            thread_right_encoder]
        for thread in list_thread:
            thread.start()
    
        print("Encoder starts testing!")

    def left_encoder(self):
        '''
        test two channels on left encoder, check whether each channel have four times of rising edge to pass the test
        '''

        while True:
            if GPIO.event_detected(self.left_pins[0]):  # update left encoder
                first_sensor = GPIO.input(self.left_pins[0]) #saved first sensor reading status
                self.left_count += 1              
                

    def right_encoder(self):
        '''
        test two channels on right encoder, check whether each channel have four times of rising edge to pass the test
        '''

        while True:
            if GPIO.event_detected(self.right_pins[0]):  # update right encoder       
                first_sensor = GPIO.input(self.right_pins[0]) #saved first sensor reading status
                self.right_count += 1                         
                if self.right_count % 4 == 0:
                    print(str(self.right_count) + ',' + str(self.left_count))
            



if __name__ == '__main__':
    encoders = Wheels_testing([35, 37], [38, 40])  # GPIO pin 35 and 37 are used for left encoder sensors
                                                    # pin 38 and 40 are used for right encoder sensors
                                                
