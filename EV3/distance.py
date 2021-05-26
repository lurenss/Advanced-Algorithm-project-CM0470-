#!/usr/bin/env python3

import sys
import signal
#import numpy as np
from time import sleep , time 
import ev3dev.ev3 as ev3

import os
os.system('setfont Lat15-TerminusBold14')

#Sensors
sensor_ultrasonic = ev3.UltrasonicSensor()
sensor_touch = ev3.TouchSensor()


#Motors
motor_left = ev3.LargeMotor('outA')
motor_right = ev3.LargeMotor('outB')






def main():
    # check that all devices are connected
    #assert all(d.connected for d in devices) is True
    
    #starting messages
    print("All devices connected")
    print()
    print("Ready to run")
    
    while True:
        distance = sensor_ultrasonic.value()/10  # convert mm to cm
        print(str(distance) + " " + "cm")


if __name__ == '__main__':
    main()