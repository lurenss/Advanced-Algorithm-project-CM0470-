#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
//#include <MeUltrasonicSensor.h>
#include <MeAuriga.h>
#include <unistd.h>
#include <math.h>
#include <time.h>


//  int time_360 = 4150; time settings on campus
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
int time_360 = 4800;
double max_distance = 200;
float heading = 0;
MeGyro gyro;

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
  gyro.begin();
  
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

/*int countNear(int distance,double *dist,float *angle){
  int count = 0;
  
  //bool flag = false;
  //unsigned long time_span[2];
  //unsigned long start;
  float first_detection;
  float second_detection;
  //unsigned long detect_time[2];
  //unsigned long start_detect;
  //unsigned long end_detect = 0;
  //double start_distance;
  //double end_distance;

  //float dist_supp;
  
  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(100);
  /*first_detection = start;
  start = millis();
  flag is used to check if an object has already been detected
  when flag is true it means that during the last rotated degree the robot already spotted an object
  the assumption is that if the robot detects objects in 2 consecutive 1 degree rotations, the object detected is the same
  //dist_supp = ultraSensor.distanceCm();
  //dist_supp = distance + 100;
  while(turn_angle(354, 'l', SPEED)){
    if(ultraSensor.distanceCm() < distance   &&   ultraSensor.distanceCm() < dist[0])
    {
      dist[0] = ultraSensor.distanceCm();
      count = 1;
      rgbled_0.setColor(count,255,0,0);
      rgbled_0.show();
      first_detection = gyro.getAngleZ();
    }
    if(dist[0] != max_distance &&  ultraSensor.distanceCm() < distance   &&   ultraSensor.distanceCm() < dist[1]){
      dist[1] = ultraSensor.distanceCm();
      count = 2;
      rgbled_0.setColor(count,255,0,0);
      rgbled_0.show();
      second_detection = gyro.getAngleZ();
    }
  }
  *angle = second_detection - first_detection;

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

  while( dist_supp > distance){
    //the robot keeps turning left until it detects the first robot
    turnLeft(SPEED);
    dist_supp = ultraSensor.distanceCm();  
    }
  //save first robot distance
  dist[count] = dist_supp; //dist[0]
  stop();
  delay(500);
  dist_supp = ultraSensor.distanceCm();
  count++;
  
  //turn on the first led to signal the first detection (color: red)
  rgbled_0.setColor(count,255,0,0);
  rgbled_0.show();
  
  
  
  while(dist_supp < distance){
    //turn right until you are not seeing the first robot anymore
    turnRight(SPEED);
    dist_supp = ultraSensor.distanceCm();
  }
  start = millis();
  Serial.print("First object detected: ");
  Serial.println(millis());
  //detect_time[0] = millis(); 
  //time_span[0] = detect_time[0] - start; //time to get to the first detection
  //start = detect_time[0];
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
  Serial.print("Second object detected: ");
  Serial.println(millis());

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
}*/

double computeHeight(double a, double c, double b){
  //far_edge=b
  double height;
  height = sqrt(pow(a,2)-pow(((pow(a,2)+pow(b,2)-pow(c,2))/(2*b)),2));
  return height;
}

double computeBase(double a, double c, double b){
  double base;
  /*Serial.print("a: ");
  Serial.print(a);
  Serial.print("c: ");
  Serial.print(c);
  Serial.print("b: ");
  Serial.print(b);*/
  
  base = (pow(a,2)+pow(b,2)-pow(c,2))/(2*b);
  return base;
}

double computeAngle(double a, double height, double base){
  double rad_angle;
  double degree_angle;
  rad_angle = acos((M_PI / 180.0)*((pow(a,2)+pow(height,2)- base) /(2*a*height))); //input to acos is an angle expressed in radiants
  Serial.print("Projected Angle (radians): ");
  Serial.print(rad_angle);
  degree_angle = rad_angle * (180/M_PI); //I want to return the angle expressed in degrees
  return degree_angle;
  }

int turn_angle(float input_angle, char rightorleft, int distance, double *dist, float *angle)
{
  int count = 0;
  float first_detection;
  float second_detection;
  
  gyro.begin(); //reset gyro


  while (heading < input_angle)
  {

    if (rightorleft == 'r')
    {
      turnRight(SPEED); /* value: between -255 and 255. */
    }
    else if (rightorleft == 'l')
    {
      turnLeft(SPEED); /* value: between -255 and 255. */
      if(ultraSensor.distanceCm() < distance   &&   ultraSensor.distanceCm() < dist[0])
      {
        dist[0] = ultraSensor.distanceCm();
        count = 1;
        rgbled_0.setColor(count,255,0,0);
        rgbled_0.show();
        first_detection = gyro.getAngleZ();
      }
      if(dist[0] != max_distance &&  ultraSensor.distanceCm() < distance   &&   ultraSensor.distanceCm() < dist[1]){
        dist[1] = ultraSensor.distanceCm();
        count = 2;
        rgbled_0.setColor(count,255,0,0);
        rgbled_0.show();
        second_detection = gyro.getAngleZ();
      }
    }
    Serial.println(gyro.getAngleZ());
    gyro.update();
    heading = gyro.getAngleZ();
    Serial.println(heading);
    }
  stop();
  *angle = second_detection - first_detection;
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
  double dist_vect[]={max_distance,max_distance}; //container for robot distances 
  float across_angle = 0;
  bool leader = false;
  if (!run_loop) return;
  int robots = 0; //set initial number of robots in the area to 0
  
  robots = turn_angle(354, 'l', 100, dist_vect, &across_angle);
  stop();
  if(robots == 2){
  //start the algorithm
  //leader election
  //compute distance between the other robots
  double score = 0;
  double far_edge = 0;
  
  far_edge = sqrt(pow(dist_vect[0],2)+pow(dist_vect[1],2)-2*dist_vect[0]*dist_vect[1]*cos(((M_PI / 180.0) * across_angle)));
  Serial.print("Far edge");
  Serial.print(far_edge);
  
  score = dist_vect[0]+dist_vect[1];
  Serial.print("\nScore");
  Serial.print(score);
  if((score > (dist_vect[0]+far_edge)) && (score > (dist_vect[1]+far_edge))){
    double projected_angle;
    double height;
    double base;
    unsigned long start_turn;
    unsigned long time_angle;
    unsigned long now;
    leader = true;
    //if leader: yellow leds
    rgbled_0.setColor(0,238,255,0);
    rgbled_0.show();
    /*check if the projection of the leader vertex over the line through the 2 non leader robots falls between the 2 non leader robots or on the side
    if one of the angles between a non leader robot and the leader robot is obtuse, then the height of the triangle falls outside the edge
    if that's the case, we move the leader towards the furthest non leader and form an isosceles triangle.
    The equal edges are of the size of the distance between the non leader robots.
    When the triangle is isosceles, we are sure that the height will fall between the 2 non leader robots, so we can procede to compute it and move the leader between
    the 2 non leader robots.
    */
    
    //compute height of triangle
    //we designed the leader to move on the perpendicular line with regard to the edge between the 2 non leader robots
    //the next function computes the lenght required to move the leader on the opposite edge
    height = computeHeight(dist_vect[0],dist_vect[1],far_edge);
    Serial.print("\nHeight");
    Serial.print(height);
    //compute the edge lenght between the closest robot and the projection of the leader on the edge of the 2 non leaders (base)
    base = computeBase(dist_vect[0],dist_vect[1],far_edge);
    Serial.print("\nBase");
    Serial.print(base);
    //next we need to compute the direction (angle in which the leader has to move to reach the opposite edge
    projected_angle = computeAngle(dist_vect[0],height,base);
    Serial.print("\nProjected angle");
    Serial.print(projected_angle);
    /*time_angle = (time_360) * (projected_angle / 360);
    start_turn = millis();
    now = start_turn;
    while(start_turn + time_angle < now){
      turnRight(SPEED);
      now = millis();
      }
     */   
  }
  else{
    //if not leader: blue leds
    rgbled_0.setColor(0,25,0,255);
    rgbled_0.show();
  }
  }
  run_loop = false;
}
