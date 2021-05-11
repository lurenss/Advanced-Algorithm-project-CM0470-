#!/usr/bin/env python3

import sys
import signal
import math 
#import numpy as np
from time import sleep , time 
import ev3dev.ev3 as ev3

import os
os.system('setfont Lat15-TerminusBold14')

#Service variables
speed = 110
tdist = 90

#Sensors
sensor_ultrasonic = ev3.UltrasonicSensor()
sensor_touch = ev3.TouchSensor()


#Motors
motor_left = ev3.LargeMotor('outB')
motor_right = ev3.LargeMotor('outA')

devices = [sensor_ultrasonic, sensor_touch, motor_left, motor_right]


#setting ultrasonic sensor in cm
sensor_ultrasonic.mode='US-DIST-CM'
units = sensor_ultrasonic.units

def stop_motors():
    motor_left.stop(stop_action="hold")
    motor_right.stop(stop_action="hold")

def start_motors(speed):
    motor_right.run_forever(speed_sp=speed)
    motor_left.run_forever(speed_sp=speed)


def rotate(speed):
    motor_right.run_forever(speed_sp=speed)
    motor_left.run_forever(speed_sp=-speed)


def scanning(speed,tdist):
    dist = []
    dist_supp = sensor_ultrasonic.value()/10 # convert mm to cm  
    output = []

    while(dist_supp > tdist):        
        rotate(speed)
        dist_supp = sensor_ultrasonic.value()/10
    
    stop_motors()
    dist.append(dist_supp)
    print("Dist is " + str(dist_supp) + " cm", file=sys.stderr)

    while(dist_supp < tdist):        
        rotate(-speed)
        dist_supp = sensor_ultrasonic.value()/10

    start = time()
    sleep(1)

    dist_supp = sensor_ultrasonic.value()/10 # convert mm to cm  
    while(dist_supp > tdist):        
        rotate(-speed)
        dist_supp = sensor_ultrasonic.value()/10
    

    sleep(1)
    elapsed_time = time() - start 
    stop_motors()
    print("Dist is " + str(dist_supp) + " cm", file=sys.stderr)
    dist.append(dist_supp)

    op_angle = elapsed_time * (360.0/7.25)
    angle_between =  360-op_angle
    print("The angle is  " + str(angle_between) + " grade", file=sys.stderr)


    #evalueting opposite side
    op_side = math.sqrt(math.pow(dist[0],2) + math.pow(dist[1],2) - 2*dist[1]*dist[0]* math.cos(math.radians(angle_between)))
    dist.append(op_side) 

    if((dist[0] + dist[1]) >  (dist[0] + dist[2]) and (dist[0] + dist[1]) >  (dist[1] +dist[2])):     #dist[2] is op_side
        ev3.Sound.speak("EV3 is the leader ciao!").wait()
        output.append(True)
        

    else:
         ev3.Sound.speak("Ev3 is not the leader!").wait()
         output.append(False)
    
    rotate(speed)
    sleep(2)

    output.append(op_angle)
    output.append(angle_between)
    output.append(op_side)
    output.append(dist)
    return output # leader, op_angle, angle_between,op_side ,vector_distances
    


def degree_360():
    out_cond = False
    motor_right.run_forever(speed_sp=110)
    motor_left.run_forever(speed_sp=-110)

    
    # service variables
    distance = sensor_ultrasonic.value()/10  # convert mm to cm
    out_cond = False

    start = time()
    end = time()
    time_window = 7.25   #7.25    360 in aula c  prop speed 110:7.2=power:time
    ts_supp = []
    ts = []
    dist_supp = []
    dist = []

    while not out_cond:
        if (end - start) > time_window:
            out_cond = True 
        
        if(distance < 100): 
            dist_supp.append(distance)
            ts_supp.append(time())
        else:
            #evaluating mean distance            
            if len(dist_supp)>0:
                sum = 0
                for x in dist_supp:  
                    sum = x + sum
                
                dist.append(int(sum/len(dist_supp)))
                dist_supp = []

            # timestamp of a detected object
            if len(ts_supp)>0:
                sum = 0
                for x in ts_supp:  
                    sum = x + sum

                ts.append(int(sum/len(ts_supp)))
                ts_supp = []

        distance = sensor_ultrasonic.value()/10  # convert mm to cm

        end = time()

    stop_motors()


    # equation derived by porpotion
    print(str(ts[1]))
    print(str(ts[0]))
    angle_between =  (ts[1] - ts[0]) * (360/(end - start))
    angle_first_obj = (ts[0] - start) * (360/(end - start))
    angle_second_obj = (ts[1] - start) * (360/(end - start))

    #evalueting opposite side
    op_side = math.sqrt(math.pow(dist[0],2) + math.pow(dist[1],2) - 2*dist[1]*dist[0]* math.cos(math.radians(angle_between))) 

    if((dist[0] + dist[1]) >  (dist[0] + op_side) and (dist[0] + dist[1]) >  (dist[1] + op_side)):
        ev3.Sound.speak("EV3 is the leader ciao!").wait()

    else:
         ev3.Sound.speak("Ev3 is not the leader!").wait()


    output = [angle_first_obj,  angle_second_obj, angle_between, op_side]
    
    return output


def move_leader(dist):
    a_sq = math.pow(dist[0],2)
    b_sq = math.pow(dist[2],2)
    c_sq = math.pow(dist[1],2)

    #this is when the robot has in front another robot with "small" distance 
    k = (a_sq+b_sq-c_sq)/(2 * dist[2])
    h = math.sqrt(a_sq - math.pow(k, 2))
    cosine = (a_sq + math.pow(h, 2) - k)/ (2 * dist[0] * h) 
    angle = math.radians(math.acos(cosine))


    #TO DO proportion for movement size
    


def main():
    # check that all devices are connected
    assert all(d.connected for d in devices) is True
    
    #starting messages
    print("All devices connected")
    print('All devices connected', file=sys.stderr)
    print()
    print("Ready to run")
    print('Ready to run', file=sys.stderr)

    #output = degree_360()
    output=scanning(speed,tdist)
    #if(output[0] == True):    this robot is the leder


'''
    motor_right.run_forever(speed_sp=speed)
    motor_left.run_forever(speed_sp=-speed)
    distance = sensor_ultrasonic.value()/10  # convert mm to cm
    print(str(distance) + " " + units)
    print("Angle first object " + str(output[0]), file=sys.stderr)
    print("Angle second object " + str(output[1]), file=sys.stderr)

    print("Angle between objects " + str(output[2]), file=sys.stderr)
    print("Op. side " + str(output[3])+ " cm", file=sys.stderr)
    sleep(5)
    '''

if __name__ == '__main__':
    main()