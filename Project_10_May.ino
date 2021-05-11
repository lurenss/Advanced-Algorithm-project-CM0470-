#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
//#include <MeUltrasonicSensor.h>
#include <MeAuriga.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#define SPEED 125
//#define SPEED 150
#define ROTATION_90 8.11
//#define ROTATION_45 4.055
//#define ROTATION_45 10
#define ROTATION_45 8
//#define ROTATION_45 30.00
#define ROTATION_180 16.22
#define ROTATION_360 32.44
#define unit_degree 1

bool run_loop = true;


MeEncoderOnBoard MotorR(SLOT1);
MeEncoderOnBoard MotorL(SLOT2);
MeLightSensor lightSensor(PORT_6);
MeUltrasonicSensor ultraSensor(PORT_10);
MeRGBLed rgbled_0(0, 12);

/*
  motors setup
*/
void setup() {
  
  rgbled_0.setpin(44);
  rgbled_0.fillPixelsBak(0, 2, 1);
  MotorR.setPulse(9); // Set pulse number of disk encoder 
  MotorL.setPulse(9);
  MotorR.setRatio(39.267); // Set encoder motor ratio 
  MotorL.setRatio(39.267);
  MotorR.setPosPid(1.8, 0, 1.2); // Set PID parameters for encoder motor position
  MotorL.setPosPid(1.8, 0, 1.2);
  MotorR.setSpeedPid(0.18, 0, 0); // Set PID parameters for encoder motor speed
  MotorL.setSpeedPid(0.18, 0, 0);
  MotorR.setMotionMode(DIRECT_MODE); // Set motor encoder to motion mode (0x00)
  MotorL.setMotionMode(DIRECT_MODE);
  Serial.begin(9600);
  lightSensor.lightOn();
  
}

void forward(int speed) {
  MotorR.setMotorPwm(-speed);
  MotorL.setMotorPwm(speed);
}

void backward(int speed) {
  Serial.print("back");
  MotorR.setMotorPwm(speed);
  MotorL.setMotorPwm(-speed);
}

void turnLeft(int speed) {
  MotorR.setMotorPwm(-speed);
  MotorL.setMotorPwm(-speed);
}

void turnRight(int speed) {
  MotorR.setMotorPwm(speed);
  MotorL.setMotorPwm(speed);
}

/* Stops the motors */
void stop() {
  MotorR.setMotorPwm(0);
  MotorL.setMotorPwm(0);
}

void rotate(int rotation, char typeTurn){ 
//rotation - > rotation degree, typeTurn -> 
//if I have to turn right or left or forward or backward R, L, B, F. If I want to perform a check rotation, to count how many robots are in the surrounding area: C.
  int end_rotate = 0;
  
  if(typeTurn == 'R'){
    while(end_rotate < rotation){
      turnRight(SPEED);
      end_rotate++;
      delay(100);
    }
    }
  if(typeTurn == 'L'){
    while(end_rotate < rotation){
      turnLeft(SPEED);
      end_rotate++;
      delay(100);
    }
  }
   Serial.print("Done turning");
   delay(200);
   stop();
   delay(200);

}

int countNear(int distance,double *dist,int *angle){
  int count = 0;
  
  //bool flag = false;
  //unsigned long time_span[2];
  unsigned long start;
  //unsigned long detect_time[2];
  //unsigned long start_detect;
  //unsigned long end_detect = 0;
  //double start_distance;
  //double end_distance;

  float dist_supp;
  
  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(100);
  //first_detection = start;
  int time_360 = 4150;
  //start = millis();
   /*flag is used to check if an object has already been detected
  when flag is true it means that during the last rotated degree the robot already spotted an object
  the assumption is that if the robot detects objects in 2 consecutive 1 degree rotations, the object detected is the same*/
  dist_supp = ultraSensor.distanceCm();
  while( dist_supp > distance){
    //the robot keeps turning left until it detects the first robot
    turnLeft(SPEED);
    dist_supp = ultraSensor.distanceCm();  
    }
  //save first robot distance
  dist[count] = dist_supp; //dist[0]

  dist_supp = ultraSensor.distanceCm();
  while(dist_supp < distance){
    turnRight(SPEED);
    dist_supp = ultraSensor.distanceCm();
  }
  //detect_time[0] = millis(); 
  //time_span[0] = detect_time[0] - start; //time to get to the first detection
  //start = detect_time[0];
  start = millis();
  
  count++;
  //turn on the first led to signal the first detection (color: red)
  rgbled_0.setColor(count,255,0,0);
  rgbled_0.show();
  //while(start < detect_time[0] + time_span[0]){
    //return to starting position without looking for another object
    //turnRight(SPEED);
    //start = millis();
    //}
  dist_supp = ultraSensor.distanceCm();
  while(dist_supp > distance){
    //back to starting position: now start looking for the second robot
    turnRight(SPEED);
    dist_supp = ultraSensor.distanceCm();
    }

  *angle = 360 - ((millis() - start)* 360/time_360);
    
  //save second robot distance
  dist[count] = dist_supp; //dist
  count++; //count = 2
  //turn on the second led to signal the second detection (color: red)
  rgbled_0.setColor(count,255,0,0);
  rgbled_0.show();
  //detect_time[1] = millis();
  //time_span[1] = detect_time[1] - detect_time[0]; //time intercurred between the 2 detections
  
  stop();
  delay(100);
  Serial.print("Objects in the area: ");
  Serial.println(count);
  Serial.print("Dist[0]: ");
  Serial.println(dist[0]);
  Serial.print("Dist[1]: ");
  Serial.println(dist[1]);
  Serial.print("Angle: ");
  Serial.println(*angle);
  return count;
}

void loop() {
  double dist_vect[]={0,0}; //container for robot distances
  int across_angle = 0;
  bool leader = false;
  if (!run_loop) return;
  int robots = 0; //set initial number of robots in the area to 0
  
  robots = countNear(150,dist_vect,&across_angle);
  stop();
  if(robots == 2){
  //start the algorithm
  //leader election
  //compute distance between the other robots
  double score = 0;
  double far_edge = 0;
  /*double l1 = pow(dist_vect[0],2);
  double l2 = pow(dist_vect[1],2);
  double stuff = 2*dist_vect[0]*dist_vect[1]*cos(across_angle);
  Serial.print("l1");
  Serial.print(l1);
  Serial.print("l2");
  Serial.print(l2);
  Serial.print("stuff");
  Serial.print(stuff);*/
  far_edge = sqrt(pow(dist_vect[0],2)+pow(dist_vect[1],2)-2*dist_vect[0]*dist_vect[1]*cos(((M_PI / 180.0) * across_angle)));
  Serial.print("Far edge");
  Serial.print(far_edge);
  /*far_edge = sqrt(l1+l2-stuff);
  Serial.print("Far edge");
  Serial.print(far_edge);*/
  
  score = dist_vect[0]+dist_vect[1];
  Serial.print("Score");
  Serial.print(score);
  if((score > (dist_vect[0]+far_edge)) && (score > (dist_vect[1]+far_edge))){
    leader = true;
    //if leader: yellow leds
    rgbled_0.setColor(0,238,255,0);
    rgbled_0.show();
  }
  else{
    //if not leader: blue leds
    rgbled_0.setColor(0,25,0,255);
    rgbled_0.show();
  }
  }
  run_loop = false;
}
