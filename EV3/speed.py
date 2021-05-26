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

    flag = True    
    while not sensor_touch.value():
        print("Ready to run")


    motor_right.run_forever(speed_sp=100)
    motor_left.run_forever(speed_sp=100)
    sleep(10)


if __name__ == '__main__':
    main()