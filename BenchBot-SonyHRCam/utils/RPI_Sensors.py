# Ultrasonic object file for creating USsensor objects and getting distance from them
import time
from tracemalloc import start

import RPi.GPIO as GPIO

# GPIO Mode (BOARD / BCM)
GPIO.setwarnings(False)
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)


class UltrasonicSensor():

    def __init__(self, trig, echo):
        self.GPIO_TRIGGER = trig
        self.GPIO_ECHO = echo
        GPIO.setup(self.GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(self.GPIO_ECHO, GPIO.IN)

    def distance(self):
        # set Trigger to HIGH and then to LOW in 0.01ms
        GPIO.output(self.GPIO_TRIGGER, True)
        time.sleep(0.00001)
        GPIO.output(self.GPIO_TRIGGER, False)
        start_time = time.time()
        stop_time = time.time()
        # save time of Echo pulse and calculate the difference
        while GPIO.input(self.GPIO_ECHO) == 0:
            start_time = time.time()
            print(f"Start time for {self.GPIO_TRIGGER}/{self.GPIO_ECHO}: {start_time}")
        while GPIO.input(self.GPIO_ECHO) == 1:
            stop_time = time.time()
            print(f"Stop time: {stop_time}")
        time_elapsed = stop_time - start_time

        # multiply with the sonic speed (34300 cm/s) then divide by 2 (to and from time)
        distance = (time_elapsed * 34300) / 2
        print(f"UltraSonic sensor distance: {distance}")
        return distance