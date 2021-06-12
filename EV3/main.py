#!/usr/bin/env python3

import sys
import signal
import math 
from time import sleep , time 
import ev3dev.ev3 as ev3

import os
os.system('setfont Lat15-TerminusBold14')

#Service variables
speed = 220    # power 70 time 12.20   power 110 time 7.55
tdist = 110    #max distance view
time_360 = 5.0    #pavimento new wheels   12.20 , 12.80 scrivania 11.80 posto x speed 70
adjust_angle = 15
adjust_length = 30
time1m = 7.50
equi_edge = 50

#Sensors
sensor_ultrasonic = ev3.UltrasonicSensor()
sensor_touch = ev3.TouchSensor()
gyro_sensor = ev3.GyroSensor()



#Motors
motor_left = ev3.LargeMotor('outC')
motor_right = ev3.LargeMotor('outB')

devices = [sensor_ultrasonic, sensor_touch, motor_left, motor_right,gyro_sensor]


#setting ultrasonic sensor in cm
#sensor_ultrasonic.mode='US-DIST-CM'
sensor_ultrasonic.mode='US-DIST-CM'
units = sensor_ultrasonic.units

#setting gyroscope mode
gyro_sensor.mode = 'GYRO-ANG'


def stop_motors():
    motor_left.stop(stop_action="hold")
    motor_right.stop(stop_action="hold")

def start_motors(speed):
    motor_right.run_forever(speed_sp=speed)
    motor_left.run_forever(speed_sp=speed)


#counterclockwise
def rotate(speed):
    motor_right.run_forever(speed_sp=speed)
    motor_left.run_forever(speed_sp=-speed)


def scanning(speed,tdist,time_360):
    dist = []
    angles = [] 
    output = []
 
    
    #sensor_ultrasonic.mode='US-SI-CM'
    sensor_ultrasonic.value()/10 # convert mm to cm 
    sensor_ultrasonic.mode='US-SI-CM'
   
    while(gyro_sensor.value() > -85):        
        rotate(speed)

    sensor_ultrasonic.mode='US-DIST-CM'
    sleep(0.5)
    dist_supp = sensor_ultrasonic.value()/10 # convert mm to cm 
    while(dist_supp > tdist):        
        rotate(speed)
        dist_supp = sensor_ultrasonic.value()/10

    #shutdowm ultrasonic
    sensor_ultrasonic.mode='US-SI-CM'
    sensor_ultrasonic.value() 

    
    stop_motors()
    dist.append(dist_supp)
    angles.append(abs(gyro_sensor.value()))
    print("Dist is " + str(dist_supp) + " cm", file=sys.stderr)
    print("First angle is " + str(angles[0]), file=sys.stderr)
    ev3.Sound.beep()
    #sensor_ultrasonic.mode='US-SI-CM'
    #dist_supp = sensor_ultrasonic.value()/10

    #start = time()
    #now = start
    while(gyro_sensor.value() < 85):        
        rotate(-speed)
        #sensor_ultrasonic.mode='US-SI-CM'
        #dist_supp = sensor_ultrasonic.value()/10
        #sleep(0.2)

    #start = time()
    #sleep(0.2)

    #sensor_ultrasonic.mode='US-SI-CM'
    #dist_supp = sensor_ultrasonic.value()/10 # convert mm to cm  
    sensor_ultrasonic.mode='US-DIST-CM'
    sleep(0.5)
    dist_supp = sensor_ultrasonic.value()/10
    while(dist_supp > tdist):        
        rotate(-speed)
        dist_supp = sensor_ultrasonic.value()/10
    
    angles.append(abs(gyro_sensor.value()))
    stop_motors()
    ev3.Sound.beep()
    
    print("Dist is " + str(dist_supp) + " cm", file=sys.stderr)
    print("Second angle is " + str(angles[1]), file=sys.stderr)
    dist.append(dist_supp)



    #op_angle = elapsed_time * (360.0/time_360)
    angle_between =  360-(angles[0]+angles[1])
    print("The angle is  " + str(angle_between) + " grade", file=sys.stderr)
    sensor_ultrasonic.mode='US-SI-CM'
    sleep(0.2)
    sensor_ultrasonic.value()
    

    #evalueting opposite side
    op_side = math.sqrt(math.pow(dist[0],2) + math.pow(dist[1],2) - 2*dist[1]*dist[0]* math.cos(math.radians(angle_between))) - 40
    print('Op side: '+ str(op_side), file=sys.stderr)

    dist.append(op_side) 

    if((dist[0] + dist[1]) >  (dist[0] + dist[2]) and (dist[0] + dist[1]) >  (dist[1] +dist[2])):     #dist[2] is op_side
        #ev3.Sound.speak("EV3 is the leader ciao!").wait()
        output.append(True)
        

    else:
         #ev3.Sound.speak("Ev3 is not the leader!").wait()
         output.append(False)
    
    #rotate(speed)
    #sleep(2)
    
    output.append(angles)
    output.append(angle_between)
    output.append(dist)
    return output # leader, op_angle, angle_between,vector_distances
    


def move_leader(dist,time_360,speed,start_degree):
    a_sq = math.pow(dist[0],2)
    b_sq = math.pow(dist[2],2)
    c_sq = math.pow(dist[1],2)

    

    #this is when the robot has in front another robot with "small" distance 
    k = (a_sq+b_sq-c_sq)/(2 * dist[2])
    h = math.sqrt(a_sq - math.pow(k, 2))
    cosine = (a_sq + math.pow(h, 2) - math.pow(k,2))/ (2 * dist[1] * h) 
    
    if cosine > 1.00 :
        cosine = 1.0

    print("Cosine is " + str(cosine) ,file=sys.stderr)

    angle = math.degrees(math.acos(cosine))

    print("Start angle is " + str(start_degree) ,file=sys.stderr)
    print("Rotation angle is " + str(angle) ,file=sys.stderr)          
    while(gyro_sensor.value() < start_degree + angle +adjust_angle ):
        rotate(-speed)
    stop_motors()

    time_to_arrive = (h+10) * 7.50/100   #5.30 is the speed to do  50 cm
    start = time() 
    while((time() - start) <time_to_arrive ):
        start_motors(360)

    stop_motors()

    if(dist[2] - k > k):
        leader_wait = time1m * ((dist[2] - k)/100)
        sleep(leader_wait)
    else:
        leader_wait = time1m * (k/100)
        sleep(leader_wait); 
  

    #end first gathering
    sleep(7)
  
    leader_move_distance = math.sqrt(math.pow(equi_edge *2,2) -math.pow(equi_edge,2))
    leader_move_time = time1m * (leader_move_distance / 100);  #time to wait  

    start = time()
    while(time()- start < leader_move_time):
        start_motors(-360) 
    
    stop_motors()
    sleep(5)

    #first triangle
    start = time()
    while(time()- start < leader_move_time):
        start_motors(360)
  
    stop_motors()

    line_deg = gyro_sensor.value() 
    print("Deg line is " + str(gyro_sensor.value()) ,file=sys.stderr)
    if(dist[0] <  dist[1]):

        while(gyro_sensor.value() > line_deg - 90):
            print("Deg line is " + str(gyro_sensor.value()) ,file=sys.stderr)
            rotate(speed)
        
        stop_motors()
        sleep(5)

        while(gyro_sensor.value() < line_deg):
            rotate(-speed)
    else:        
        while(gyro_sensor.value() < line_deg + 90):
            print("Deg line is " + str(gyro_sensor.value()) ,file=sys.stderr)
            rotate(-speed)
        
        stop_motors()    
        sleep(5)
        
        while(gyro_sensor.value() > line_deg):
            rotate(speed)

  
    
    stop_motors()

    sleep(5)



    #first line
    leader_wait = time1m * ((equi_edge/2)/100)
    sleep(leader_wait)

    #second gathering

    start = time()
    while(time()- start < leader_move_time):
        start_motors(-360)
  
    stop_motors()
    sleep(7)

    #second triangle

    start = time()
    while(time()- start < leader_move_time):
        start_motors(360)
    
    stop_motors()
    
    #second line
    line_deg = gyro_sensor.value() 
    print("Deg line is " + str(gyro_sensor.value()) ,file=sys.stderr)
    if(dist[0] > dist[1]):

        while(gyro_sensor.value() > line_deg - 90):
            print("Deg line is " + str(gyro_sensor.value()) ,file=sys.stderr)
            rotate(speed)
        
        stop_motors()
        sleep(5)

    else:        
        while(gyro_sensor.value() < line_deg + 90):
            print("Deg line is " + str(gyro_sensor.value()) ,file=sys.stderr)
            rotate(-speed)
        
        stop_motors()    
        sleep(5)
        

def press_button_start():
    while(not sensor_touch.value()):
        print('Waiting')    

def move_not_leader(dist):

    if(dist[1] > dist[0]):
        s_edge = dist[0]
        l_edge = dist[1]
        #turnAngle(*out_angle - ADJUST_ANGLE)    
  
    else:
        s_edge = dist[1]
        l_edge = dist[0]
        start_degree = gyro_sensor.value()
        print("Start degreee is"+str(gyro_sensor.value()), file=sys.stderr)
        while(gyro_sensor.value() < (start_degree + 10)):
            print(str(gyro_sensor.value()), file=sys.stderr)
            rotate(-360)
        
        stop_motors()
        
        sleep(3.5)


    sleep(5)
    
    k = abs((math.pow(l_edge,2) + math.pow(s_edge,2) - math.pow(dist[2],2)) / (2 * s_edge))
    print('K: '+ str(k), file=sys.stderr)


    h = math.sqrt(pow(l_edge,2) - pow(k,2))
    print('H: '+ str(h), file=sys.stderr)



    wait_time = time1m * ((h+10)/100)
    print('Wait_time: '+ str(wait_time), file=sys.stderr)


    sleep(wait_time)

    if(k > (s_edge/2)):
        k = (s_edge/2) - 15

    move_time = time1m * ((k)/100)
    
    start = time()
    while(time() - start < move_time):
        start_motors(360)

    stop_motors()

    sleep(2)
    leader_move_distance = math.sqrt(pow(equi_edge,2) - math.pow(equi_edge/2,2))
    leader_move_time = time1m * (leader_move_distance / 100)  

    move_time = time1m * ((equi_edge/2)/100)
    start = time()
    while(time()- start < move_time):
        start_motors(-360)
    
    stop_motors()

    if(leader_move_time - move_time > 0):
        sleep(leader_move_time - move_time)  

    sleep(2)
    sleep(leader_move_time)
    sleep(2)

    #first line
  
    start = time()
    while(time()- start < move_time):
        start_motors(360)

    stop_motors()

    sleep(2)

    start = time()
    while(time()- start < move_time):
        start_motors(-360)

    stop_motors()
    if(leader_move_time - move_time > 0):
        sleep(leader_move_time - move_time)  

    sleep(2)

     
    sleep(leader_move_time)


def main():
    # check that all devices are connected
    assert all(d.connected for d in devices) is True
    
    #starting messages
    print("All devices connected")
    print('All devices connected', file=sys.stderr)
    print()
    print("Ready to run")
    print('Ready to run', file=sys.stderr)

    press_button_start()
    sleep(7.8)

    #output = degree_360()
    output=scanning(speed,tdist,time_360)
    
    start_degree = output[1]
    start_degree = start_degree[1]
    if(output[0] == True):    #this robot is the leader

        move_leader(output[3],time_360,speed,start_degree)
    
    else:
        #not leader case
        dist = output[3]
        move_not_leader(dist)

            
if __name__ == '__main__':
    main()