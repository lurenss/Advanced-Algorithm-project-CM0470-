#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
//#include <MeUltrasonicSensor.h>
#include <MeAuriga.h>
#include <unistd.h>
#include <math.h>
#include <time.h>


//  int time_360 = 4800; time settings on campus
#define SPEED 35
//#define SPEED 150
#define ROTATION_90 8.11
//#define ROTATION_45 4.055
//#define ROTATION_45 10
#define ROTATION_45 8
//#define ROTATION_45 30.00
#define ROTATION_180 16.22
#define ROTATION_360 32.44
#define unit_degree 1
#define ADJUST_ANGLE 30
#define ADJUST_LENGTH 30
//#define equi_edge 50

bool run_loop = true;
int time_360 = 5700;
float time1M = 7500;
MeGyro gyro (0,0x69);
float equi_edge = 50.0;

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
  //Serial.print("back");
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


  



int scan(int distance, float *dist, float *out_angle)
{
  int count = 0;
  float first_detection;
  float second_detection;
  float dist_supp;
  float op_side;
  float angle;
  unsigned long start;
  unsigned long actual;
  float leader_move_distance;
  float leader_move_time;
  

  dist_supp = ultraSensor.distanceCm();
  while(dist_supp > distance){
    turnLeft(SPEED); /* value: between -255 and 255. */
    dist_supp = ultraSensor.distanceCm();
    gyro.update();
  }
  stop();
  dist[0] = dist_supp;  
  count++;
  rgbled_0.setColor(count,255,0,0);
  rgbled_0.show();
  gyro.update();
  first_detection = gyro.getAngleZ();
  Serial.print("\nfirst detection: ");
  Serial.println(first_detection);


  start = millis();
  actual = start;
  while((actual - start) < 1000){
    actual = millis();
    turnRight(SPEED);
  }
  stop();
  dist_supp = ultraSensor.distanceCm();
  while(dist_supp > distance){
    turnRight(SPEED); /* value: between -255 and 255. */
    dist_supp = ultraSensor.distanceCm();
    gyro.update();
  }
  stop();
  dist[1] = dist_supp;
  count++;
  rgbled_0.setColor(count,255,0,0);
  rgbled_0.show();
  gyro.update();
  second_detection = gyro.getAngleZ();
  Serial.print("\nsecond detection: ");
  Serial.println(second_detection);
    
  angle = 360 - (abs(first_detection) + abs(second_detection));
  *out_angle = angle;

  delay(100);
  Serial.print("Objects in the area: ");
  Serial.println(count);
  Serial.print("Dist[0]: ");
  Serial.println(dist[0]);
  Serial.print("Dist[1]: ");
  Serial.println(dist[1]);
  Serial.print("Angle: ");
  Serial.println(angle);
  

  dist[2] = sqrt(pow(dist[0],2) + pow(dist[1],2) - 2*dist[0]*dist[1] * cos(((M_PI / 180.0) * angle)));
  dist[2] = dist[2] - ADJUST_LENGTH;
  
  Serial.print("Far edge: ");
  Serial.println(dist[2]);

  if((dist[0] + dist[1]) >  (dist[0] + dist[2]) && (dist[0] + dist[1]) >  (dist[1] +dist[2])){   
    rgbled_0.setColor(0,238,255,0);
    rgbled_0.show();
    return true;
  }
  else{
    //if not leader: blue leds
    rgbled_0.setColor(0,25,0,255);
    rgbled_0.show();
    return false;    
  }    
}

void turnAngle(float angle){
  float heading = 0;
  gyro.begin();
  while (abs(heading) < angle){
    turnRight(SPEED);
    gyro.update();
    heading = gyro.getAngleZ();
  }
  stop();
  Serial.println("Done turning");
}

void move_leader(float dist[]){
  float k;
  float h;
  float cosine;
  float angle_deg;
  float start;
  float a_sq = pow(dist[0],2);
  float b_sq = pow(dist[2],2);
  float c_sq = pow(dist[1],2);
  unsigned long leader_wait;
  float leader_move_distance;
  unsigned long leader_move_time;

  k = (a_sq+b_sq-c_sq)/(2 * dist[2]);
  h = sqrt(a_sq - pow(k, 2));
  cosine = (a_sq + pow(h, 2) - pow(k,2))/ (2 * dist[1] * h); 
  
  if (cosine > 1.00){
     cosine = 1.0;
  }
  if (cosine < -1.00){
     cosine = -1.0;
  }
  
  Serial.print("\nCosine:");
  Serial.print(cosine);
  angle_deg = (acos(cosine)) * (180.0/3.141592653589793238463);
  turnAngle(angle_deg- ADJUST_ANGLE/3);

  //Move the leader
  float moving_time = time1M * ((h+10)/100);
  float start_forward = millis();
  float now = start_forward; //set now smaller than start_turn + moving_time to permit the loop to start
  while(start_forward + moving_time > now){
      forward(SPEED);
      now = millis();
  }
  stop(); 
  
  if(dist[2] - k > k) {
    leader_wait = time1M * ((dist[2] - k)/100);
    delay(leader_wait); 
  }
  else{
    leader_wait = time1M * (k/100);
    delay(leader_wait); 
  }

  //end first gathering
  delay(2000);
  
  leader_move_distance = sqrt(pow(equi_edge*2,2) - pow(equi_edge,2));
  leader_move_time = time1M * (leader_move_distance / 100);  //time to wait  

  start = millis();
  while(millis()- start < leader_move_time){
    backward(SPEED);
  }
  stop();
  delay(2000);

  //first triangle
  start = millis();
  while(millis()- start < leader_move_time){
    forward(SPEED);
  }
  stop();
  delay(2000);

  //first line
  leader_wait = time1M * ((equi_edge/2)/100);
  delay(leader_wait);

  //second gathering

  start = millis();
  while(millis()- start < leader_move_time){
    backward(SPEED);
  }
  stop();
  delay(2000);

  //second triangle

  start = millis();
  while(millis()- start < leader_move_time){
    forward(SPEED);
  }
  stop();
  //second line
 
}  


void not_leader(float dist[], float* out_angle){
  float s_edge;
  float l_edge;
  float k;
  float h;
  unsigned long wait_time;
  unsigned long move_time;
  unsigned long leader_move_time;
  float leader_move_distance;
  unsigned long start;

  if(dist[1] > dist[0]){
    s_edge = dist[0];
    l_edge = dist[1];
    turnAngle(*out_angle - ADJUST_ANGLE);    
  }
  else{
    s_edge = dist[1];
    l_edge = dist[0];
    turnAngle(10);
  }

  delay(5000);
  k = abs((pow(l_edge,2) + pow(s_edge,2) - pow(dist[2],2)) / (2 * s_edge));
  Serial.println("\n K: ");
  Serial.println(k);

  h = sqrt(pow(l_edge,2) - pow(k,2));


  wait_time = time1M * ((h+10)/100);
  Serial.println("\n wait_time: ");
  Serial.println(wait_time);
  
  Serial.println("\n H: ");
  Serial.println(h);

  delay(wait_time);

  if(k > (s_edge/2)){
    k = (s_edge/2) - 15;
  }
  move_time = time1M * ((k)/100);
  
  
  
  start = millis();
  while(millis()- start < move_time){
    forward(SPEED);
  }
  stop();

  delay(2000);
  leader_move_distance = sqrt(pow(equi_edge*2,2) - pow(equi_edge,2));
  leader_move_time = time1M * (leader_move_distance / 100);  //time to wait

  move_time = time1M * ((equi_edge/2)/100);
  start = millis();
  while(millis()- start < move_time){
    backward(SPEED);
  }
  stop();

  if(leader_move_time - move_time > 0){
    delay(leader_move_time - move_time);  
  }

  delay(2000);

  delay(leader_move_time);

  delay(2000);

  //first line
  
  start = millis();
  while(millis()- start < move_time){
    forward(SPEED);
  }
  stop();

  delay(2000);

 start = millis();
 while(millis()- start < move_time){
    backward(SPEED);
  }
  stop();

  if(leader_move_time - move_time > 0){
    delay(leader_move_time - move_time);  
  }

  delay(2000);

  delay(leader_move_time);
 
}

void loop() {
  float dist_vect[3]; //container for robot distances 
  bool leader;
  float out_angle;

  delay(5000);
  gyro.begin();
  if (!run_loop) return;
  
  gyro.update();
  Serial.read();
  leader = scan(100, dist_vect, &out_angle);

  if(leader == true){
    move_leader(dist_vect);
  }
  else{
    not_leader(dist_vect, &out_angle); 
   }

   while(1){}
   
}
  
