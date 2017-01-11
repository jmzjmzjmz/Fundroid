#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

void SetCalibrationData(adafruit_bno055_offsets_t &calibData)
{
  calibData.accel_offset_x = 20;
  calibData.accel_offset_y = 65454;
  calibData.accel_offset_z = 0;

  calibData.gyro_offset_x = 0;
  calibData.gyro_offset_y = 65534;
  calibData.gyro_offset_z = 65535;

  calibData.mag_offset_x = 65354;
  calibData.mag_offset_y = 221;
  calibData.mag_offset_z = 65323;

  calibData.accel_radius = 1000;
  calibData.mag_radius = 839;
}

void WaitForCalibrationComplete()
{
  int numberConsecutiveReads = 3;
  int numReads = 0;
  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;

  while(numReads != numberConsecutiveReads)
  {
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print("Continue when Mag == 3... Currently Mag=");
    Serial.println(mag, DEC);
    delay(BNO055_SAMPLERATE_DELAY_MS);

    if(mag == 3)
    {
      numReads++;
    }
    else
    {
      numReads = 0;
    }
  }
  
}

/*
  Wheels are 96 steps per rotation 
  Now there are 192 since we have 6 pulses per internal motor revolution
  Wheel Diameter: 253mm
  Wheel Circumference: 796mm, maybe different though
  distance per step: 8.29mm  
  Wheelbase = 502mm
*/
int inByte = 0;         // incoming serial byte

const int dirPinR = 8;
const int spdPinR = 9;
const int dirPinL = 10;
const int spdPinL = 11;
const int resetButton = 12;
const int led = 13;
int topSpeed = 70;
int bottomSpeed = 30;

boolean goingStraight = 0;

int rWheelTicks = 0;
int lWheelTicks = 0;
int rWheelChange = 0;         //for calculating change since last sample
int lWheelChange = 0;

int rWheelSpeed = 0;
int lWheelSpeed = 0;

float rWheelDist = 0;
float lWheelDist = 0;
float distCenter = 0;

float botAngle = 0;
float angleChange = 0;

float botX = 0;
float botY = 0;

float changeX = 0;
float changeY = 0;

// 32:1 Gear Ratio
float wheelBase = 494.5;
float wheelCircumference = 784;     //based on measuring w a wire (was originally 796)
float numOfSteps = 192;
float stepDist = wheelCircumference/numOfSteps;

int rHitTarget = 0;
int lHitTarget = 0;

const byte interruptPin1 = 2;
const byte interruptPin2 = 3;
int rTarget = -10;
int lTarget = -10;

boolean inMotion = false;
boolean targetSet = false;

void setup() {

  Serial.begin(9600);
 
 
  //Motor Controls
  pinMode (dirPinR, OUTPUT);
  pinMode (spdPinR, OUTPUT);
  analogWrite(spdPinR, 0);
  pinMode (dirPinL, OUTPUT);
  pinMode (spdPinL, OUTPUT);
  analogWrite(spdPinL, 0);
  digitalWrite(dirPinR, HIGH);
  digitalWrite(dirPinL, HIGH);

  //Encoder Read Pins
  pinMode(2, INPUT_PULLUP); //  Opto Reader 1    (RIGHT)
  pinMode(4, INPUT);        //  Opto Reader 2    (RIGHT)
  pinMode(3, INPUT_PULLUP); //  Opto Reader 3    (LEFT)
  pinMode(5, INPUT);        //  Opto Reader 4    (LEFT)
  attachInterrupt(digitalPinToInterrupt(interruptPin1), rRising, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), lRising, CHANGE);

  pinMode(resetButton, INPUT_PULLUP);
  pinMode(led, OUTPUT);

  //  Serial.println("press grey button to start...");
  //
  //while(digitalRead(13) == HIGH){
  //  delay(100);
  //}

//  Serial.println("starting");
digitalWrite(led, HIGH);
  delay(500);
  digitalWrite(led, LOW);
  delay(5000);
}

int curMove = 0;

void loop() {
//setWheelSpeeds();
//printInfo();            //nice looking display w labels
//infoToProcessing();       //CSV for processing

  if(curMove == 0)
  {
    // 245 STEPS GOES 1.02M
    //GoToPoint(245);

    //goToAngle(180);

    GoToPoint(490);
    // Serial.println("Go to Angle 30");
    // goToAngle(30);
  }
  else if(curMove == 1)
  {
    goToAngle(180);
  }
  else if(curMove == 2)
  {
    GoToPoint(490);
  }
  else if(curMove == 3)
  {
    goToAngle(360);
  }
  // else if(curMove == 4)
  // {
  //   GoToPoint(100);
  // }
  // else if(curMove == 5)
  // {
  //   goToAngle(270);
  // }
  // else if(curMove == 6)
  // {
  //   GoToPoint(100);
  // }
  // else if(curMove == 7)
  // {
  //   goToAngle(360);
  // }
  // else if(curMove == 8)
  // {
  //   GoToPoint(100);
  // }

  calcAngleCoordinates();

  if(!inMotion)
  {
    targetSet = false;
    curMove++;
    Serial.print("Cur Angle: ");
    Serial.print(botAngle*57.3);
    Serial.print(" On to move: ");
    Serial.println(curMove);
    delay(1000);
  }

}

int marginOfError = 1;
boolean IsAngleAcceptable(int a)
{
    
  if(a+1 > botAngle*57.3 && a-1 < botAngle*57.3)
  {
    inMotion = false;
    return true;
  }

  return false;
}

float kP = 0.1;
int GetSpeedValue(int curTicks, int destTicks)
{

  int error = destTicks - curTicks;
  int spd = error*kP;
  if(spd < 0)
    spd = -1*spd;

  if(spd < 30)
    spd = 30;
  else if(spd > 50)
    spd = 50;

  return spd;
}

void goToAngle(int a){ 

  Serial.println(botAngle*57.3);

  if(IsAngleAcceptable(a))
  {
    digitalWrite(dirPinR, LOW);               //FWD
    analogWrite(spdPinR, 0);
    digitalWrite(dirPinL, HIGH);               //FWD
    analogWrite(spdPinL, 0);
    inMotion = false;
    return;
  }

  int wheelTicks = GetDesiredWheelTicks(a);
  Serial.println(wheelTicks);

  int spd1 = GetSpeedValue(rWheelTicks ,wheelTicks);
  int spd2 = GetSpeedValue(lWheelTicks, wheelTicks);
  Serial.print("spd1 ");
  Serial.print(spd1);
  Serial.print(" spd2 ");
  Serial.println(spd2);

  digitalWrite(dirPinR, LOW);               //FWD
  analogWrite(spdPinR, GetSpeedValue(rWheelTicks ,wheelTicks));
  digitalWrite(dirPinL, HIGH);               //FWD
  analogWrite(spdPinL, GetSpeedValue(lWheelTicks ,wheelTicks));
  inMotion = true;
   
//    
//
//  // Rotate CCW
//  if(botAngle*57.3 < a){
//    int wheelTicks = GetDesiredWheelTicks(a);
//    
//    digitalWrite(dirPinR, LOW);               //FWD
//    analogWrite(spdPinR, GetSpeedValue();
//    digitalWrite(dirPinL, HIGH);               //FWD
//    analogWrite(spdPinL, 50);
//    inMotion = true;
//  }
//  // Rotate CW
//  else if(botAngle*57.3 > a){
//    digitalWrite(dirPinR, HIGH);               //FWD
//    analogWrite(spdPinR, 50);
//    digitalWrite(dirPinL, LOW);               //FWD
//    analogWrite(spdPinL, 50);
//    inMotion = true;
//  }
  
}

void GoToPoint(int point)
{
  if(!targetSet)
  {
    rTarget = point+rWheelTicks;
    lTarget = point+lWheelTicks;
    targetSet = true;
  }
  goToTarget();
}


void rRising() {
  if (digitalRead(2) == HIGH){
  if (digitalRead(4) == HIGH) {
    rWheelTicks--;
    rWheelChange--;
  }
  else {
    rWheelTicks++;
    rWheelChange++;
  }
  }
  else{
     if (digitalRead(4) == HIGH) {
    rWheelTicks++;
    rWheelChange++;
  }
  else {
    rWheelTicks--;
    rWheelChange--;
  }
  }
}

void lRising() {
  if (digitalRead(3) == HIGH){
   if (digitalRead(5) == HIGH) {
    lWheelTicks--;
    lWheelChange--;
   }
   else {
    lWheelTicks++;
    lWheelChange++;
    }
   }
  else{
   if (digitalRead(5) == HIGH) {
    lWheelTicks++;
    lWheelChange++;
   }
   else {
    lWheelTicks--;
    lWheelChange--;
    }
  }
}

void setWheelSpeeds(){
  rWheelSpeed = map(abs(rTarget - rWheelTicks), 100, 0, topSpeed, bottomSpeed);       // these need to change (the mapping from 100-0 is not correct
  lWheelSpeed = map(abs(lTarget - lWheelTicks), 100, 0, topSpeed, bottomSpeed);
  rWheelSpeed = constrain(rWheelSpeed, bottomSpeed, topSpeed);
  lWheelSpeed = constrain(lWheelSpeed, bottomSpeed, topSpeed);
}


float fkP = 0.7;
int GetForwardSpeedValue(int curTicks, int destTicks)
{

  int error = destTicks - curTicks;
  int spd = error*fkP;
  if(spd < 0)
    spd = -1*spd;

  if(spd < 30)
    spd = 30;
  else if(spd > 100)
    spd = 100;

  return spd;
}

void goToTarget(){
  inMotion = true;

  int rWheelSpeed = GetForwardSpeedValue(rWheelTicks, rTarget);

  int rDiff = abs(rTarget - rWheelTicks);
  int lDiff = abs(lTarget - lWheelTicks);

  float ikp = 0.5;
  int lrDiff = rDiff - lDiff;
  if(lrDiff > 0) // Add to r speed because r is further away
  {
    rWheelSpeed += ikp*lrDiff;
  }
  

  if (rWheelTicks <rTarget) {
    rHitTarget = 0;
    digitalWrite(dirPinR, LOW);               //FWD
    analogWrite(spdPinR, rWheelSpeed);
  }
  else if (rWheelTicks > rTarget) {
    rHitTarget = 0;
    digitalWrite(dirPinR, HIGH);              //REV
    analogWrite(spdPinR, rWheelSpeed);
  }
  else {
    analogWrite(spdPinR, 0);
    rHitTarget = 1;
  }

  int lWheelSpeed = GetForwardSpeedValue(lWheelTicks, lTarget);
  if(lrDiff < 0)
  {
    lWheelSpeed += ikp*abs(lrDiff);
  }

  if (lWheelTicks < lTarget) {
    lHitTarget = 0;
    digitalWrite(dirPinL, LOW);
    analogWrite(spdPinL, lWheelSpeed);
  }
  else if (lWheelTicks > lTarget) {
    lHitTarget = 0;
    digitalWrite(dirPinL, HIGH);
    analogWrite(spdPinL, lWheelSpeed);
  }
  else {
    analogWrite(spdPinL, 0);
    lHitTarget = 1;
  }

  if(lHitTarget && rHitTarget)
  {
    inMotion = false;
  }

  Serial.print("RWheelSpeed :");
  Serial.print(rWheelSpeed);
  Serial.print(" LWheelSPeed :");
  Serial.print(lWheelSpeed);
  Serial.print(" LWheelTicks :");
  Serial.print(lWheelTicks);
  Serial.print(" rWheelTicks :");
  Serial.print(rWheelTicks);
  Serial.print(" Diff :");
  Serial.println(lrDiff);
}


void newTarget(int x){
  if (lHitTarget + rHitTarget == 2) {
    rTarget = random(-x, x);
    lTarget = random(-x, x);
  //  Serial.println(lTarget);
    rHitTarget = 0;
    lHitTarget = 0;
    delay(500);
  }
}

int GetDesiredWheelTicks(float deltaAngle)
{
  deltaAngle = botAngle*57.3 - deltaAngle;
  
  int desiredWheelTicks = ((deltaAngle*wheelBase)/2) / stepDist;

  return desiredWheelTicks;
}

void calcAngleCoordinates() {

  rWheelDist = rWheelChange * stepDist;
  lWheelDist = lWheelChange * stepDist;
  angleChange = (rWheelDist - lWheelDist) / wheelBase;
  
  botAngle = botAngle + angleChange;
  distCenter = (rWheelDist + lWheelDist)/2; 
  botX = botX + distCenter*cos(botAngle);
  botY = botY + distCenter*sin(botAngle);

  
  rWheelChange = 0;
  lWheelChange = 0;
  angleChange = 0;

  

}

void printInfo() {
  Serial.print("     lTarget: ");
  Serial.print(lTarget);
    Serial.print("  rTarget: ");
  Serial.print(rTarget);
  Serial.print("   lWhlChange: ");
  Serial.print(lWheelChange);

  Serial.print("   rWhlChange: ");
  Serial.print(rWheelChange);

  Serial.print("  botAngle(degrees): ");
  Serial.print((botAngle*57.3));
  Serial.print("    botX: ");
  Serial.print(botX/100);
  Serial.print("    botY: ");
  Serial.print(botY/100);
  
  Serial.println();
}

void infoToProcessing(){

    Serial.print(botX/100, DEC);
    Serial.print(",");
    Serial.print(botY/100, DEC);
    Serial.print(",");
    Serial.println((botAngle*57.3), DEC);
 
}

