// right wheel forward: right_1 HIGH & right_2 LOW & right_pwm
// left wheel forward: left_1 HIGH & left_2 LOW & left_pwm
// side_sonar_front: trigPin1 & echoPin1
// side_sonar_back: trigPin2 & echoPin2
// front_sonar_centre: trigPin3 & echoPin3
// front_sonar_left: trigPin4 & echoPin4
// front_sonar_right: trigPin5 & echoPin5
// colour_sensor: colour_sensor_pin

//gps angle: angle(gpsArray[1],gpsArray[2])
//gps distance: distance()
//gps target: lat_target , lon_target

#define target_one 41
#define target_two 43
#define target_three 42

#define colour_sensor_pin A12
#define colour_sensor_pin_L A9
#define colour_sensor_pin_C A10
#define colour_sensor_pin_R A11
#define right_1 7 
#define left_2 4
#define right_2 8
#define left_1 9
#define right_pwm 5
#define left_pwm 6
#define trigPin1 49 //side front
#define echoPin1 48
#define trigPin2 23 //side back 
#define echoPin2 22
#define trigPin4 46 //left
#define echoPin4 47
#define trigPin3 27  //centre
#define echoPin3 26
#define trigPin5 45  //right
#define echoPin5 44

#define GPS_BAUD 9600
#define N_doubleS 4

double lat_target = 0; //6.799984 79.901908
double lon_target = 0;

double lat_start = 6.7981797;
double lon_start = 79.8990145;

int shoulder = 50; //arm pins
int elbow = 51;
int base = 52;
int pulsewidth;
int magnet = 53;
int k;

#include <Wire.h>
#include <HMC5883L.h>
#include "RunningAverage.h"
#include "Ublox.h"
#include "math.h"
#include "PID.h"

HMC5883L compass;
Ublox M8_Gps;
RunningAverage gpsRA_lat(100);
RunningAverage gpsRA_lon(100);
RunningAverage compassRA(5);
double gpsArray[N_doubleS] = {0, 0, 0, 0};  // Altitude - Latitude - Longitude - N Satellites
double compass_angle = 0;
bool notArrivedAtFF;
bool detectedCircle = false;
bool reachedWhite = false;
bool letsGoHome = false;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(GPS_BAUD);

  pinMode( target_one, INPUT);
  pinMode( target_two, INPUT);
  pinMode( target_three, INPUT);
  pinMode( colour_sensor_pin, INPUT);
  pinMode( colour_sensor_pin_L, INPUT);
  pinMode( colour_sensor_pin_C, INPUT);
  pinMode( colour_sensor_pin_R, INPUT);
  pinMode(right_1, OUTPUT);
  pinMode(right_2, OUTPUT);
  pinMode(left_1, OUTPUT);
  pinMode(left_2, OUTPUT);
  pinMode(right_pwm, OUTPUT);
  pinMode(left_pwm, OUTPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);
  pinMode(trigPin5, OUTPUT);
  pinMode(echoPin5, INPUT);
  Wire.begin();

  pinMode(shoulder,OUTPUT); //arm pins
  pinMode(elbow,OUTPUT);
  pinMode(base, OUTPUT);
  pinMode(magnet, OUTPUT);

  compass = HMC5883L(); //new instance of HMC5883L library
  setupHMC5883L(); //setup the HMC5883L
  select_target();
}


void loop() {

//  search();
//  delay(10000000000);

//  reach_pit();



  go_to_FF();
  if (detectedCircle){
    find_white();
  }
  if (reachedWhite) {
    reach_pit();
    }
  if (letsGoHome) {
    return_home();
  }
  
  


//  Serial.println(distance(),6);
//  Serial.println(gpsArray[1],6);
//  Serial.println(gpsArray[2],6);
//  Serial.println(gpsRA_lat.getAverage(),6);
//  Serial.println(gpsRA_lon.getAverage(),6);
//  run_gps();
//  Serial.println(" ");  

//  wall_following(15);

//  obstacle(40);

//  Serial.println(side_sonar_back()); 
//  Serial.println(front_sonar_left()); 
//  Serial.println(front_sonar_centre()); 


//  Serial.print("Angle: ");
//  Serial.println(angle(gpsArray[1],gpsArray[2]));
//  Serial.print("Distance: ");
//  Serial.println(distance());
//  Serial.println(lat_target,8);

  //white_front();

}


void go_to_FF() {
  notArrivedAtFF = true;
  while (notArrivedAtFF) {
    obstacle(50);
    if (detectedCircle) {
      break;
    }
  }
}


void find_white() {
  long start1=millis();
  while (detectedCircle) {
    lineFollow();
    if ( (millis()-start1) > 1000) {
          start1=millis();
          if ( side_sonar_front()<100 ) {
            turn_by(-90);
            long start2=millis();
            while (true){
              reachObstacle(150);
              if ( (millis()-start2) > 500) {
                start2=millis();
                if (front_sonar_centre()<6) {
                  moto_stop();
                  if (white_front()){
                    reachedWhite = true;
                    detectedCircle=false; 
                    break;                                     
                  }
                  else{ //at black one
                    turn_by(90);
                    while(~bottomWhite()) {
                      moto_forward(100);
                    }
                    moto_stop();
                    break;
                  }
                }
              }
            }
          }          
    }
  }
}
          

void reach_pit() {
  while ( front_sonar_centre()>12){
    moto_forward(100);
    delay(40);
  }
  turn_by(90);
  wall_following(15);
  search();
  letsGoHome = true;
}


void return_home() {
  turn_by(-90);
  moto_forward(100);
  delay(500);
  moto_stop();
  turn_by(90);
  moto_forward(100);
  delay(2000);
  lat_target = lat_start;
  lon_target = lon_start;
  notArrivedAtFF = true;
  while (notArrivedAtFF) {
    obstacle(50);
    if (detectedCircle) {
      break;
    }
  }  
    
}

//all integrated obstacle avoidance code
void obstacle(int thresh) {
  int i=0;
  if (front_sonar_left() > thresh && front_sonar_right() < thresh) {
    do {
      left(100);
      delay(50);
      Serial.println("left1");
    } while (front_sonar_right() < thresh);
  }
  else if (front_sonar_centre() < thresh && front_sonar_right() > thresh) {
    moto_stop();
    do {
      right(100);
      delay(50);
      Serial.println("right1");
    } while (front_sonar_left() < thresh);
  }
  else if (front_sonar_left() < thresh && front_sonar_centre() > thresh) {
    moto_stop();
    do {
      right(100);
      
      delay(50);
      Serial.println("right2");
    } while (front_sonar_left() < thresh);
  }
  else if (front_sonar_left() < thresh && front_sonar_centre() < thresh && front_sonar_right() < thresh) {
    moto_stop();
    do {
      moto_reverse();
      delay(20);
      Serial.println("reverse");
    } while (front_sonar_centre() < (5));
      moto_stop();
    do {
      right(100);
      delay(50);
      Serial.println("right3");
    } while (front_sonar_left() < thresh);
  }
  else if (front_sonar_left() > thresh && front_sonar_centre() > thresh && front_sonar_right() > thresh) {
    do {
      moto_forward(100);
      i=i+1;
      if (i>100000000000){ //10^^11
        turn_to_angle(2);
        i=0;
        if (distance()<5){ //figure out the threshold here. 5 is random.
          moto_stop();
          detectedCircle= bottomWhite();
          if (detectedCircle){
            notArrivedAtFF= false;
            break;
          }
        }
        if (distance()<2){
          notArrivedAtFF= false;
          break;
        }
      }
    }
      while (front_sonar_left() > thresh && front_sonar_centre() > thresh && front_sonar_right() > thresh);
      Serial.println("forward");
  }
  //  else {
  //    moto_forward(100);
  //    Serial.println("forward");
  //  }
}


void wall_following(int thresh){
  while (true){

    if (side_sonar_front()>thresh){
//      moto_forward(100);
//      delay(200);
      if(side_sonar_back()>20) {          
        moto_forward(100);
        delay(200);
        moto_stop();
        turn_by(-80);
        long start=millis();
        while(true) {
          moto_forward(100);
          delay(40);
          if (front_sonar_centre()<14) {
            moto_stop();
            turn_by(-90);
            moto_stop();
            break;
          }
          if ( (millis()-start)>600){
            moto_stop();
            turn_by(-80);
            moto_stop();
            break;            
          }
        }
        break;
//        moto_forward(100);
//        delay(500);
//        turn_by(-90);
//        break;
      }
    }
    
    if (side_sonar_front()>thresh){
    
        digitalWrite(right_1, HIGH);
        digitalWrite(right_2, LOW);
        digitalWrite(left_1, LOW);
        digitalWrite(left_2, LOW);
        analogWrite(right_pwm, 100);
        analogWrite(left_pwm, 80);
        delay(500);
        moto_stop();
        Serial.println("left");

    }
    else{

        moto_forward(90);
        delay(300);
        moto_stop();
        Serial.println("forward");

    }
  }
}


// runs gps and gets current position from 100 long running average (5s delay to calculate this). Turns to the target angle. 
void turn_to_angle(int thresh) {

  double target=angle();
 
  while (true) {
    double error=getHeading()-target; //angles measured clockwise from North
    while (error>180){
      error-=360;
    }
    while (error<-180){
      error+=360;
    }
    
    Serial.println(target,6);
    Serial.println(getHeading(),6);
    Serial.println(error,6);
    Serial.println(" ");
    
    if (error>150 || error<-150){
      right(100);
    }
    else if (error>thresh) {
      left(100);
    }
    else if (error<-thresh) {
      right(100);
    }
    else if (error<thresh || error>-thresh) {
      moto_stop();
      break;      
    }
  }
}


// turns angle clockwise
void turn_by(double angle) {

  double thresh=1;
  double target=getHeading()+angle;
 
  while (true) {
    double error=getHeading()-target; //angles measured clockwise from North
    while (error>180){
      error-=360;
    }
    while (error<-180){
      error+=360;
    }
    
    Serial.println(target,6);
    Serial.println(getHeading(),6);
    Serial.println(error,6);
    Serial.println(" ");
    
    if (error>150 || error<-150){
      right(100);
    }
    else if (error>thresh) {
      left(100);
    }
    else if (error<-thresh) {
      right(100);
    }
    else if (error<thresh || error>-thresh) {
      moto_stop();
      break;      
    }
  }
}


// colour sensor for detecting front white/black
bool white_front() {
  int sensorValue = analogRead(colour_sensor_pin);
  if (sensorValue < 50) {
    return true;
  }
  else {
    return false;
  }
}


// setting up compass. how does this communicate? port?
void setupHMC5883L() {
  //Setup the HMC5883L, and check for errors
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
}


//figure out how this works and document
double getHeading() {
  Vector norm = compass.readNormalize();

  // Calculate heading
  double heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  double declinationAngle =-1.565 ;
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  double headingDegrees = heading * 180/M_PI; 
  return headingDegrees;
}


//returns angle: clockwise from N.
double angle() {

  run_gps();
  
//  gpsRA_lat.clear();
//  gpsRA_lon.clear();  
//
//  long start=millis();
//  while( (millis()-start)<2000){
//    run_gps();
//  }
//  double lat_n = gpsRA_lat.getAverage() * M_PI/180; //conversion to rad
//  double lon_n = gpsRA_lon.getAverage() * M_PI/180;
  
  double lat_n = gpsArray[1]* M_PI/180;
  double lon_n = gpsArray[2]* M_PI/180 ;
  double lat_t = lat_target * M_PI/180;
  double lon_t = lon_target * M_PI/180;

  Serial.println(lat_n,6);
  Serial.println(lon_n,6);
  Serial.println(lat_t,6);
  Serial.println(lon_t,6);  
  
  double lon_dif = lon_t - lon_n; //dx
  
  double theta = atan2(sin(lon_dif)*cos(lon_t), cos(lat_n)*sin(lat_t) - sin(lat_n)*cos(lat_t)*cos(lon_dif)) ; //atan2 function considers quadrants issue

  theta=theta*180/M_PI;
  
  if (theta<0){
    theta=theta+360;
  }
  return 360-theta;
}


// returns distance to target
double distance() {
  
  run_gps();
  
//  gpsRA_lat.clear();
//  gpsRA_lon.clear();  
//
//  long start=millis();
//  while( (millis()-start)<2000){
//    run_gps();
//  }
//  double lat_n = gpsRA_lat.getAverage() * M_PI/180; //conversion to rad
//  double lon_n = gpsRA_lon.getAverage() * M_PI/180;
  
  double lat_n = gpsArray[1];
  double lon_n = gpsArray[2];
  double lat_t = lat_target * M_PI/180;
  double lon_t = lon_target * M_PI/180;

//  Serial.println(lat_n,6);
//  Serial.println(lon_n,6);
//  Serial.println(lat_t,6);
//  Serial.println(lon_t,6);
    
  double lat_dif = lat_t - lat_n;
  double lon_dif = (lon_t - lon_n)*cos((lat_t + lat_n)/2);
  double dist = sqrt(pow(lat_dif, 2) + pow(lon_dif, 2))*6371000;
  return dist; //6371000 Earth Radius
}


// updates gps coordinates in gpsArray. refer to function code for details.  
void run_gps() {
  if (!Serial1.available())
    return;

  while (Serial1.available()) {
    char c = Serial1.read();
    if (M8_Gps.encode(c)) {
      gpsArray[0] = M8_Gps.altitude;
      gpsArray[1] = M8_Gps.latitude;
      gpsArray[2] = M8_Gps.longitude;
      gpsArray[3] = M8_Gps.sats_in_use;
      gpsRA_lat.addValue(M8_Gps.latitude);
      gpsRA_lon.addValue(M8_Gps.longitude);
    }
  }
//  for (byte i = 0; i < N_doubleS; i++) {
//    Serial.print(gpsArray[i], 6);
//    Serial.print(" ");
//  }
//  Serial.println(" ");
}


void moto_stop() {
  digitalWrite(right_1, HIGH);
  digitalWrite(left_2, HIGH);
  digitalWrite(right_2, HIGH);
  digitalWrite(left_1, HIGH);
  analogWrite(right_pwm, 0);
  analogWrite(left_pwm, 0);
  delay(500);
  digitalWrite(right_1, LOW);
  digitalWrite(left_2, LOW);
  digitalWrite(right_2, LOW);
  digitalWrite(left_1, LOW);
}


void right(int speed) {
  digitalWrite(right_1, LOW);
  digitalWrite(right_2, HIGH);
  digitalWrite(left_1, HIGH);
  digitalWrite(left_2, LOW);
  analogWrite(right_pwm, speed);
  analogWrite(left_pwm, speed-20);
}

void right_slow(int speed) {
  for (int i=0:255){
    while (i<speed){  
      digitalWrite(right_1, LOW);
      digitalWrite(right_2, HIGH);
      digitalWrite(left_1, HIGH);
      digitalWrite(left_2, LOW);
      analogWrite(right_pwm, 0);
      analogWrite(left_pwm, 100);
    }
  }
}

void left(int speed) {
  digitalWrite(right_1, HIGH);
  digitalWrite(right_2, LOW);
  digitalWrite(left_1, LOW);
  digitalWrite(left_2, HIGH);
  analogWrite(right_pwm, speed);
  analogWrite(left_pwm, speed-20);
}

void left_slow(int speed) {
  for (int i=0:255){
    while (i<speed){  
      digitalWrite(right_1, HIGH);
      digitalWrite(right_2, LOW);
      digitalWrite(left_1, LOW);
      digitalWrite(left_2, HIGH);
      analogWrite(right_pwm, 120);
      analogWrite(left_pwm, 0);
    }
  }
}


void moto_forward(int speed) {
  // lowest speed: 40
  // recommended lowest: 70
  
  digitalWrite(right_1, HIGH);
  digitalWrite(right_2, LOW);
  digitalWrite(left_1, HIGH);
  digitalWrite(left_2, LOW);
  analogWrite(right_pwm, speed);
  analogWrite(left_pwm, speed - 20);
}


void moto_reverse() {
  digitalWrite(right_1, LOW);
  digitalWrite(right_2, HIGH);
  digitalWrite(left_1, LOW);
  digitalWrite(left_2, HIGH);
  analogWrite(right_pwm, 120);
  analogWrite(left_pwm, 100);
}


double side_sonar_front() {
  double duration, distance;
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration = pulseIn(echoPin1, HIGH);
  distance = duration / 58.2;

  return distance;
}


long side_sonar_back() {
  long duration, distance;
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration = pulseIn(echoPin2, HIGH);
  distance = duration / 58.2;

  return distance;
}


long front_sonar_centre() {
  long duration, distance;
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  duration = pulseIn(echoPin3, HIGH);
  distance = duration / 58.2;

  return distance;
}


long front_sonar_left() {
  long duration, distance;
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  duration = pulseIn(echoPin4, HIGH);
  distance = duration / 58.2;

  return distance;
}


long front_sonar_right() {
  long duration, distance;
  digitalWrite(trigPin5, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin5, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin5, LOW);
  duration = pulseIn(echoPin5, HIGH);
  distance = duration / 58.2;

  return distance;
}

void lineFollow() {
  bool sensorR = analogRead(colour_sensor_pin_R)<50;
  bool sensorC = analogRead(colour_sensor_pin_C)<50;
  bool sensorL = analogRead(colour_sensor_pin_L)<50;

  if (sensorL && sensorC && sensorR) {
    right(100);
  }
  else if (sensorC && sensorR) {
    moto_forward(100);
  }
  else if (sensorL && sensorC) {
    left(100);
  }
  else if (sensorC){
    moto_forward(100);
  }
  else {
    left(100);
  }
}


bool bottomWhite() {
  return (analogRead(colour_sensor_pin_R)<50 && analogRead(colour_sensor_pin_C)<50 && analogRead(colour_sensor_pin_L)<50 );
}


void reachObstacle(int thresh) {
  bool sensorR = front_sonar_right()<thresh;
  bool sensorC = front_sonar_centre()<thresh;
  bool sensorL = front_sonar_left()<thresh;

  if (sensorL && sensorC && sensorR) {
    moto_forward(100);
  }
  else if (sensorC && sensorR) {
    right(100);
  }
  else if (sensorL && sensorC) {
    left(100);
  }
  else if (sensorL) {
    left(100);
  }
  else if (sensorC) {
    moto_forward(100);
  }
  else if (sensorR) {
    right(100);
  }
  else {
    left(100);
  }
}

void select_target() {
  if  (digitalRead(target_one) == HIGH){
    lat_target = 6.7980259;
    lon_target = 79.8989084;
  }
  else if  (digitalRead(target_two) == HIGH){
    lat_target = 6.7982046;
    lon_target = 79.8991565;
  }
  else if  (digitalRead(target_three) == HIGH){ //right switch
    lat_target = 6.7983451;
    lon_target = 79.8989933;
  }
}


void servo(int servoPin,int angle){
  pulsewidth = angle*10+600;
  digitalWrite(servoPin,HIGH);
  delayMicroseconds(pulsewidth);
  digitalWrite(servoPin,LOW);
  }


void magnetOn(){
  digitalWrite(magnet,HIGH);  
}


void magnetOff(){
  digitalWrite(magnet,LOW);
}


void search(){
  magnetOn();
  k=0;
int l=0;
while(l<100){
  servo(shoulder,115);
  servo(elbow,140);
  servo(base,-10);
  delay(20);
  l+=1;
}
l=0;
while(l<10){
  servo(shoulder,115);
  servo(elbow,140-l);
  servo(base,-10);
  delay(20);
  l+=1;
}
k=115;
while(k<=130){
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k-=4;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k-=2;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k+=1;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
}
k=-10;
while(k<20){
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=3;  
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=4;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=2;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=1;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
}
k=130;
while(k>=90){
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k+=4;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k+=2;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=1;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);  
}
k=20;
while(k<=100){
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k-=4;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k-=2;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k+=1;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  servo(shoulder,90);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  servo(shoulder,90);
  servo(elbow,90);
  servo(base,k);
  delay(20);
}
while(k>=60){
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k+=4;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k+=2;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  k-=1;
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);
  servo(shoulder,89);
  servo(elbow,90);
  servo(base,k);
  delay(20);  
}
l=0;
while(l<=15){
  servo(shoulder,95);
  servo(elbow,90+l);
  servo(base,k);
  delay(20);
  l+=2;
  servo(shoulder,95);
  servo(elbow,90+l);
  servo(base,k);
  delay(20);
  l-=1;
  servo(shoulder,95);
  servo(elbow,90+l);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,90+l);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,90+l);
  servo(base,k);
  delay(20);
}
while(k>=40){
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k+=4;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k+=2;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=1;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
}
while(k<=80){
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=4;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=2;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k+=1;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
}
while(k>=60){
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k+=4;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k+=2;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  k-=1;
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115);
  servo(base,k);
  delay(20);
}
l=0;
while(l<=35){
  servo(shoulder,95);
  servo(elbow,115-l);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115-l);
  servo(base,k);
  delay(20);
  l+=1;
  servo(shoulder,95);
  servo(elbow,115-l);
  servo(base,k);
  delay(20);
  servo(shoulder,95);
  servo(elbow,115-l);
  servo(base,k);
  delay(20);  
}
l=0;
while(l<=4){
  servo(shoulder,95-l);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,95-l);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,95-l);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  l+=1;
}
while(k<=80){
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k-=4;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k-=2;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=1;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
}
while(k>=40){
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=4;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=2;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k-=1;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
}
while(k<=60){
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k-=4;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k-=2;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  k+=1;
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
  servo(shoulder,91);
  servo(elbow,80);
  servo(base,k);
  delay(20);
}
////////////////////////////////////////////////////////////////
k=91;
while(k<=130){
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  k-=4;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  k-=2;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  k+=1;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  }
k=60;
while(k>=-13){
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=4;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=2;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=1;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
}
k=0;
while(k<=15){
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  k+=2;
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  k-=3;
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  k+=2;
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
}
l=0;
while(l<=100){  
  servo(shoulder,115);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  l+=1;
}
magnetOff();
}
void place(){
   magnetOn();
  k=0;
int l=0;
while(l<100){
  servo(shoulder,115);
  servo(elbow,140);
  servo(base,-10);
  delay(20);
  l+=1;
}
l=0;
while(l<10){
  servo(shoulder,115);
  servo(elbow,140-l);
  servo(base,-10);
  delay(20);
  l+=1;
}
k=115;
while(k<=130){
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k-=4;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k-=2;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  k+=1;
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
  servo(shoulder,k);
  servo(elbow,130);
  servo(base,-10);
  delay(20);
}
k=-10;
while(k<20){
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=3;  
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=4;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=3;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=2;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=1;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
}
k=130;
while(k>=90){
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k+=4;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k+=2;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=1;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);  
}
l=0;
magnetOff();
k=91;
while(k<=130){
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=4;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k+=3;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k-=2;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  k+=1;
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,60);
  delay(20);
  servo(shoulder,k);
  servo(elbow,k);
  servo(base,20);
  delay(20);
  }
k=20;
while(k>=-13){
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=4;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=3;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k+=2;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  k-=1;
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
  servo(shoulder,130);
  servo(elbow,130);
  servo(base,k);
  delay(20);
}
k=0;
while(k<=15){
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  k+=2;
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  k-=3;
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  k+=2;
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
  servo(shoulder,130-k);
  servo(elbow,130);
  servo(base,-13);
  delay(20);
}
l=0;
while(l<=100){  
  servo(shoulder,115);
  servo(elbow,140);
  servo(base,-13);
  delay(20);
  l+=1;
}
}

