#!/usr/bin/env python3

import sys
import math 
from time import sleep, time
import ev3dev.ev3 as ev3
import os

os.system('setfont Lat15-TerminusBold14')

# Service variables
speed = 220 # Robot speed
tdist = 110 # Maximum distance view of the robot (in cm)
time_360 = 5.0 # Time to rotate 360 degrees
adjust_angle = 15
adjust_length = 30
time1m = 7.50 # Time to travel 1 meter
equi_edge = 50 # Fixed to 50 cm for space between robots
continuous_measurement = 'US-DIST-CM' # mode for continuous measurement in centimeters
single_measurement = 'US-SI-CM' # mode for single measurement in centimeters

# Sensors setup
sensor_ultrasonic = ev3.UltrasonicSensor()
sensor_touch = ev3.TouchSensor()
gyro_sensor = ev3.GyroSensor()

# Motors setup
motor_left = ev3.LargeMotor('outC')
motor_right = ev3.LargeMotor('outB')

devices = [sensor_ultrasonic, sensor_touch, motor_left, motor_right, gyro_sensor]

# Set ultrasonic sensor mode
sensor_ultrasonic.mode = continuous_measurement
units = sensor_ultrasonic.units

# Set gyroscope mode
gyro_sensor.mode = 'GYRO-ANG'

def press_button_start():
    while(not sensor_touch.value()):
        print('Waiting')

def stop_motors():
    motor_left.stop(stop_action="hold")
    motor_right.stop(stop_action="hold")

def start_motors(speed):
    motor_right.run_forever(speed_sp=speed)
    motor_left.run_forever(speed_sp=speed)

def rotate(speed):
    # Positive speed = rotate counterclockwise
    # Negative speed = rotate clockwise
    motor_right.run_forever(speed_sp=speed)
    motor_left.run_forever(speed_sp=-speed)

def scanning(speed, tdist):
    dist = [] # List of the distances/edge lengths of the triangle that the 3 robots create between them
    angles = [] # List of the angles of the triangle that the 3 robots create
    output = [] # Output list to return at the end of the method

    sensor_ultrasonic.value()/10 # Convert mm to cm
    sensor_ultrasonic.mode = single_measurement

    # Rotate the robot counterclockwise until it sees the first robot
    # Keep sensor off until it's passed 85 degrees to avoid interference
    while(gyro_sensor.value() > -85):
        rotate(speed)

    # Turn on ultrasonic sensor
    sensor_ultrasonic.mode = continuous_measurement
    sleep(0.5)
    dist_supp = sensor_ultrasonic.value()/10 # convert mm to cm

    # Rotate the robot until it sees the first robot
    while(dist_supp > tdist):        
        rotate(speed)
        dist_supp = sensor_ultrasonic.value()/10

    # Shut down ultrasonic sensor upon seeing the first robot
    sensor_ultrasonic.mode = single_measurement
    sensor_ultrasonic.value()  # Take measurement for first robot observed
    stop_motors()

    # Save information from first measurement
    dist.append(dist_supp) # Add first distance/triangle edge length detected to the list
    angles.append(abs(gyro_sensor.value())) # Add first angle detected to the list, depending on how much the robot had to rotate

    # Print output for first measurement & alert with beep
    print("Dist is " + str(dist[0]) + " cm", file=sys.stderr)
    print("First angle is " + str(angles[0]), file=sys.stderr)
    ev3.Sound.beep()

    # Rotate clockwise until it sees the second robot
    # Keep sensor off until it's passed 85 degrees to avoid interference
    while(gyro_sensor.value() < 85):        
        rotate(-speed)

    # Turn on ultrasonic sensor
    sensor_ultrasonic.mode = continuous_measurement
    sleep(0.5)
    dist_supp = sensor_ultrasonic.value()/10 # convert mm to cm

    # Rotate the robot until it sees the second robot
    while(dist_supp > tdist):        
        rotate(-speed)
        dist_supp = sensor_ultrasonic.value()/10

    stop_motors()

    # Save information from second measurement
    dist.append(dist_supp) # Add second distance detected to the list
    angles.append(abs(gyro_sensor.value())) # Add second angle detected to the list, depending on how much the robot had to rotate

    # Print output for second measurement & alert with beep
    print("Dist is " + str(dist[1]) + " cm", file=sys.stderr)
    print("Second angle is " + str(angles[1]), file=sys.stderr)
    ev3.Sound.beep()

    # Calculate the third angle of the triangle
    angle_between = 360 - (angles[0] + angles[1])
    print("The angle is " + str(angle_between) + " grade", file=sys.stderr)

    # shutdown ultrasonic sensor
    sensor_ultrasonic.mode = single_measurement
    sleep(0.2)
    sensor_ultrasonic.value()

    # Calculate the third edge distance of the triangle (the opposite side between the 2 observed robots)
    op_side = math.sqrt(math.pow(dist[0], 2) + math.pow(dist[1], 2) - 2 * dist[1] * dist[0] * math.cos(math.radians(angle_between))) - 40
    dist.append(op_side)
    print('Op side: ' + str(op_side), file=sys.stderr)

    # LEADER ELECTION: Determine whether or not the robot is the leader
    # Robot is the leader if the sum of its observed distances are greater than the sum of the other robots' observed distances
    total_observed_dist = dist[0] + dist[1]
    r1_observed_dist = dist[0] + dist[2]
    r2_observed_dist = dist[1] + dist[2]
    if((total_observed_dist > r1_observed_dist) and (total_observed_dist) > (r2_observed_dist)):
        output.append(True)
    else:
        output.append(False)

    output.append(angles) # List of angles
    output.append(angle_between) # Calculated angle between the two observed robots
    output.append(dist) # List of distances

    return output # leader, list of angles, angle_between, list of distances

# Controls movement of the robot if it was determined to be the leader
def move_leader(dist, speed, start_degree):
    a_sq = math.pow(dist[0], 2)
    b_sq = math.pow(dist[2], 2)
    c_sq = math.pow(dist[1], 2)

    # When the robot has another robot in front of it with a "small" distance
    # Trig calculations behind how to move towards the robots to gather together
    k = (a_sq+b_sq-c_sq)/(2 * dist[2])
    h = math.sqrt(a_sq - math.pow(k, 2))
    cosine = (a_sq + math.pow(h, 2) - math.pow(k,2))/ (2 * dist[1] * h) 
    
    if cosine > 1.00:
        cosine = 1.0

    angle = math.degrees(math.acos(cosine))

    print("Cosine is " + str(cosine), file=sys.stderr)
    print("Start angle is " + str(start_degree), file=sys.stderr)
    print("Rotation angle is " + str(angle), file=sys.stderr)

    # Rotate the robot towards the other robots
    totalDegrees = start_degree + angle + adjust_angle
    while(gyro_sensor.value() < totalDegrees):
        rotate(-speed)

    stop_motors()

    # Calculate the time it will take to travel to the other robots
    time_to_arrive = (h+10) * 7.50/100
    start = time() 
    while((time() - start) < time_to_arrive ):
        start_motors(360)

    stop_motors()

    # Wait for the other robots to meet together
    if(dist[2] - k > k):
        leader_wait = time1m * ((dist[2] - k)/100)
        sleep(leader_wait)
    else:
        leader_wait = time1m * (k/100)
        sleep(leader_wait)

    sleep(7)
    # END FIRST GATHERING

    # Calculate how much to move to ensure distance is equal between the robots
    leader_move_distance = math.sqrt(math.pow(equi_edge *2, 2) -math.pow(equi_edge, 2))
    leader_move_time = time1m * (leader_move_distance / 100)  # time to wait

    start = time()

    # Move backwards once you've waited the right amount of time
    while(time() - start < leader_move_time):
        start_motors(-360)

    # Stop in triangle formation
    stop_motors()
    sleep(5)

    # END FORMING FIRST TRIANGLE

    # Wait and then move forward in between the other robots to make a straight line
    start = time()
    while(time() - start < leader_move_time):
        start_motors(360)
  
    stop_motors()

    # Calculate which direction/how much to turn the robot so it faces the same direction in the line
    line_deg = gyro_sensor.value() 
    print("Deg line is " + str(gyro_sensor.value()), file=sys.stderr)

    if(dist[0] < dist[1]):
        while(gyro_sensor.value() > line_deg - 90):
            print("Deg line is " + str(gyro_sensor.value()), file=sys.stderr)
            rotate(speed)
        
        stop_motors()
        sleep(5)

        while(gyro_sensor.value() < line_deg):
            rotate(-speed)
    else:        
        while(gyro_sensor.value() < line_deg + 90):
            print("Deg line is " + str(gyro_sensor.value()), file=sys.stderr)
            rotate(-speed)
        
        stop_motors()    
        sleep(5)
        
        while(gyro_sensor.value() > line_deg):
            rotate(speed)
    
    stop_motors()
    sleep(5)

    # END FORMING FIRST STRAIGHT LINE

    # Waits for other robots to turn and move towards each other
    leader_wait = time1m * ((equi_edge/2)/100)
    sleep(leader_wait)

    # END SECOND GATHERING

    # Wait and then move to form second triangle
    start = time()
    while(time()- start < leader_move_time):
        start_motors(-360) # move backwards
  
    stop_motors()
    sleep(7)

    # END FORMING OF SECOND TRIANGLE

    # Wait and then move forward in between the other robots to make a straight line
    start = time()
    while(time()- start < leader_move_time):
        start_motors(360) # move forward
    
    stop_motors()

    # Calculate which direction/how much to turn the robot so it faces the same direction in the line
    line_deg = gyro_sensor.value() 
    print("Deg line is " + str(gyro_sensor.value()), file=sys.stderr)
    if(dist[0] > dist[1]):

        while(gyro_sensor.value() > line_deg - 90):
            print("Deg line is " + str(gyro_sensor.value()), file=sys.stderr)
            rotate(speed)
        
        stop_motors()
        sleep(5)

    else:        
        while(gyro_sensor.value() < line_deg + 90):
            print("Deg line is " + str(gyro_sensor.value()), file=sys.stderr)
            rotate(-speed)
        
        stop_motors()    
        sleep(5)

    # END FORMING SECOND LINE

# Controls movement if robot is determined to not be the leader
def move_not_leader(dist):
    # Determine on which side of the leader the robot is, and set the edges accordingly
    if(dist[1] > dist[0]):
        s_edge = dist[0]
        l_edge = dist[1]
    else:
        s_edge = dist[1]
        l_edge = dist[0]

    # Get starting angle degree
    start_degree = gyro_sensor.value()
    print("Start degree is" + str(gyro_sensor.value()), file=sys.stderr)

    # Rotate the robot towards the other robots
    while(gyro_sensor.value() < (start_degree + 10)):
        rotate(-360)
        
    stop_motors()
    sleep(5)

    # Trig calculations behind how to move towards the robots to gather together
    k = abs((math.pow(l_edge, 2) + math.pow(s_edge, 2) - math.pow(dist[2], 2)) / (2 * s_edge))
    h = math.sqrt(pow(l_edge, 2) - pow(k, 2))
    wait_time = time1m * ((h+10)/100)

    print('K: ' + str(k), file=sys.stderr)
    print('H: ' + str(h), file=sys.stderr)
    print('Wait_time: ' + str(wait_time), file=sys.stderr)

    sleep(wait_time)

    # Calculate how much/long to move to meet the other robots
    if(k > (s_edge/2)):
        k = (s_edge/2) - 15

    move_time = time1m * ((k)/100)
    
    start = time()
    while(time() - start < move_time):
        start_motors(360)

    stop_motors()
    sleep(2)

    # END FIRST GATHERING

    # Calculate how much/for how long the leader will be moving
    leader_move_distance = math.sqrt(pow(equi_edge, 2) - math.pow(equi_edge/2, 2))
    leader_move_time = time1m * (leader_move_distance / 100)  

    # Calculate how much to move to form a triangle
    move_time = time1m * ((equi_edge/2)/100)
    start = time()
    while(time() - start < move_time):
        start_motors(-360) # move backwards
    
    stop_motors()

    # END FORMING FIRST TRIANGLE

    # Wait for leader to move to form a straight line
    if(leader_move_time - move_time > 0):
        sleep(leader_move_time - move_time)  

    sleep(4)
    sleep(leader_move_time)

    # END FIRST STRAIGHT LINE

    # Wait and then move towards other robots to gather again
    start = time()
    while(time()- start < move_time):
        start_motors(360)

    stop_motors()
    sleep(2)

    # END SECOND GATHERING

    # Wait and then move to form another triangle
    start = time()
    while(time()- start < move_time):
        start_motors(-360) # move backwards

    stop_motors()

    # END FORMING SECOND TRIANGLE

    # Wait until the leader moves to form a straight line
    if(leader_move_time - move_time > 0):
        sleep(leader_move_time - move_time)  

    sleep(2)
    sleep(leader_move_time)

    # END FORMING SECOND LINE

def main():
    # Check that all devices are connected
    assert all(d.connected for d in devices) is True
    
    # Print starting messages
    print("All devices connected")
    print('All devices connected', file=sys.stderr)
    print()
    print("Ready to run")
    print('Ready to run', file=sys.stderr)

    # Start the robots
    press_button_start()
    sleep(7.8)

    # Scan the initial configuration of the robots
    output = scanning(speed, tdist, time_360) # output = leader, list of angles, angle_between, list of distances

    # Set variables according to output of scan
    start_degree = output[1]
    start_degree = start_degree[1]

    if(output[0] == True):
        # This robot is the leader
        move_leader(output[3], time_360, speed, start_degree)
    
    else:
        # This robot is not the leader
        dist = output[3]
        move_not_leader(dist)

if __name__ == '__main__':
    main()