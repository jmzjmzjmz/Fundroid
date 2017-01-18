/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  GetDistanceI2c

  This example shows how to initialize, configure, and read distance from a
  LIDAR-Lite connected over the I2C interface.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND
  
  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/
#include <Bounce.h>
#include <Wire.h>
#include <LIDARLite.h>
#include <math.h>

#define MOTORCOMMPORT Serial1

#define REAL double
#define MAX_READS 600

inline static REAL sqr(REAL x) {
    return x*x;
}

int linreg(int n, REAL x[], REAL y[], REAL* m, REAL* b, REAL* r)
{
    REAL   sumx = 0.0;                        /* sum of x                      */
    REAL   sumx2 = 0.0;                       /* sum of x**2                   */
    REAL   sumxy = 0.0;                       /* sum of x * y                  */
    REAL   sumy = 0.0;                        /* sum of y                      */
    REAL   sumy2 = 0.0;                       /* sum of y**2                   */

   for (int i=0;i<n;i++)   
      { 
      sumx  += x[i];       
      sumx2 += sqr(x[i]);  
      sumxy += x[i] * y[i];
      sumy  += y[i];      
      sumy2 += sqr(y[i]); 
      } 

   REAL denom = (n * sumx2 - sqr(sumx));
   if (denom == 0) {
       // singular matrix. can't solve the problem.
       *m = 0;
       *b = 0;
       if (r) *r = 0;
       return 1;
   }

   *m = (n * sumxy  -  sumx * sumy) / denom;
   *b = (sumy * sumx2  -  sumx * sumxy) / denom;
   if (r!=NULL) {
      *r = (sumxy - sumx * sumy / n) /          /* compute correlation coeff     */
            sqrt((sumx2 - sqr(sumx)/n) *
            (sumy2 - sqr(sumy)/n));
   }

   return 0; 
}

LIDARLite myLidarLite;

int dirPin = 2;
int stepPin = 3;
int stepTime = 10;
int curStep = 0;

int optoPin = 11;
double angleOffset = 0;

double sweepTo = 0.0;
double sweepFrom = 0.0;

int num_Readings = 0;

boolean FoundZero = false;
boolean SweepDone = false;
boolean SweepError = false;

#define staticDataSet 600
#define StepsPerRotation 400

struct LidarRead{
  double angle;
  int reading;
};

int curDataPoints = 0;
LidarRead dataPoints[staticDataSet];

double GetAngleFromStep(int step)
{
  double floatStep = step;
  return 360.0*(floatStep/StepsPerRotation) + angleOffset;
}

/////////////////////////////////////////////
//////// FUNNIE MOTOR FEET CONTROLS /////////
int stepsPerMeter = 240;
void MoveMotorForward(float meters)
{
  int stepsToMove = meters*stepsPerMeter;

  String First = "1,";
  String last = "&";
  String MoveForward = First + stepsToMove + last;

  MOTORCOMMPORT.println(MoveForward);
  Serial.print("Sending: ");
  Serial.println(MoveForward);
}

int curSystemAngle = 0;
void MoveMotorToAngle(int angle)
{
  String First = "0,";
  String last = "&";
  String MoveAngle = First + angle + last;

  MOTORCOMMPORT.println(MoveAngle);
  Serial.print("Sending: ");
  Serial.println(MoveAngle);

  curSystemAngle = angle;
}

boolean CheckForMotionComplete()
{
  if(MOTORCOMMPORT.available())
  {
    if(MOTORCOMMPORT.read() == '0')
    {
      return true;
    }
  }

  return false;
}

void FLUSHMOTORBUFFER()
{
  while(MOTORCOMMPORT.available())
  {
    MOTORCOMMPORT.read();
  }
}
////////////////////////////////////////////
////////////////////////////////////////////

void setup()
{
  delay(1000);
  
  MOTORCOMMPORT.begin(9600);
  Serial.begin(115200); // Initialize serial connection to display distance readings

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  
  digitalWrite(dirPin, 0);
  digitalWrite(stepPin, 0);
  
  pinMode(optoPin, INPUT);
  attachInterrupt(optoPin, fullRotation, FALLING); // interrrupt 1 is data ready


  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
//  pinMode(14, OUTPUT);
//  digitalWrite(14, HIGH);
  
  /*
    begin(int configuration, bool fasti2c, char lidarliteAddress)

    Starts the sensor and I2C.

    Parameters
    ----------------------------------------------------------------------------
    configuration: Default 0. Selects one of several preset configurations.
    fasti2c: Default 100 kHz. I2C base frequency.
      If true I2C frequency is set to 400kHz.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  
  myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

  /*
    configure(int configuration, char lidarliteAddress)

    Selects one of several preset configurations.

    Parameters
    ----------------------------------------------------------------------------
    configuration:  Default 0.
      0: Default mode, balanced performance.
      1: Short range, high speed. Uses 0x1d maximum acquisition count.
      2: Default range, higher speed short range. Turns on quick termination
          detection for faster measurements at short range (with decreased
          accuracy)
      3: Maximum range. Uses 0xff maximum acquisition count.
      4: High sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for high sensitivity and noise.
      5: Low sensitivity detection. Overrides default valid measurement detection
          algorithm, and uses a threshold value for low sensitivity and noise.
    lidarliteAddress: Default 0x62. Fill in new address here if changed. See
      operating manual for instructions.
  */
  myLidarLite.configure(0); // Change this number to try out alternate configurations

  sweepTo = 180;
  sweepFrom = 90;

}

void BinarySearchForBestLine(REAL x[], REAL y[], int n, double acceptableRSquared)
{
  REAL x1[MAX_READS];
  REAL x2[MAX_READS];
  REAL y1[MAX_READS];
  REAL y2[MAX_READS];

  int n1 = n/2;
  for(int i = 0; i<n/2; i++)
  {
    x1[i] = x[i];
    y1[i] = y[i];
  }

  REAL m1,b1,r1, rSquared1;
  rSquared1 = 0.0;
  int lineStatus = linreg(n1,x1,y1,&m1,&b1,&r1);
  rSquared1 = r1*r1;

  int n2 = n - n/2;
  for(int i = n/2; i<n; i++)
  {
    x2[i-n/2] = x[i];
    y2[i-n/2] = y[i];
  }

  REAL m2,b2,r2, rSquared2;
  rSquared2 = 0.0;
  lineStatus = linreg(n2,x2,y2,&m2,&b2,&r2);
  rSquared2 = r2*r2;

  if(rSquared1 > rSquared2)
  {
    sweepFrom = sweepFrom;
    sweepTo = (sweepTo-sweepFrom)/2 + sweepFrom;
  }
  else
  {
    sweepFrom = (sweepTo-sweepFrom)/2 + sweepFrom;
    sweepTo = sweepTo;
  }

  Serial.print("SweepTo: ");
  Serial.print(sweepTo);
  Serial.print(" SweepFrom: ");
  Serial.println(sweepFrom);
}

double FindBestFitLineInDataSet(REAL x[], REAL y[], int n, int WallShouldBeOnRight)
{
  int acceptableNumberOfReads = 3;
  double acceptableRSquared = 0.90;
  double outlierThreshold = 30.0;
  int maxCycles = 5;
  REAL newX[MAX_READS];
  REAL newY[MAX_READS];

  REAL m,b,r, rSquared;
  rSquared = 0.0;
  int lineStatus = linreg(n,x,y,&m,&b,&r);

  if(lineStatus == 1)
  {
    SweepError = true;
  }
  else
  {
    double Angle = atan(m)*180/3.14;
    rSquared = r*r;

    Serial.print("Angle: ");
    Serial.print(Angle);
    Serial.print(" rSquared: ");
    Serial.println(rSquared);

    if(rSquared > acceptableRSquared)
    {
      Serial.println("LINE DETECTED");
      SweepDone = true;
    }
    else
    {
      BinarySearchForBestLine(x, y, n, acceptableRSquared);
    }
  }

  return rSquared;
}

void stepForward()
{
  digitalWrite(dirPin, HIGH);
  digitalWrite(stepPin, HIGH);

  delay(stepTime/2);
  digitalWrite(stepPin, LOW);
  delay(stepTime/2);

  curStep++;
}

void stepBackward()
{
  digitalWrite(dirPin, LOW);
  digitalWrite(stepPin, HIGH);

  delay(stepTime/2);
  digitalWrite(stepPin, LOW);
  delay(stepTime/2);

  curStep--;
}

void Initialize()
{
  FoundZero = false;
  while(!FoundZero)
  {
    stepBackward();
  }

  curStep = 0;
}

double GetRadiansFromDegrees(double Degrees)
{
  return (Degrees/180)*3.14159;
}

void PrintSweepInfo()
{
  for(int i = 0; i<curDataPoints; i++)
  {
    double radians = GetRadiansFromDegrees(dataPoints[i].angle);
    double radius = dataPoints[i].reading;

    double x = radius*cos(radians);
    double y = radius*sin(radians);

    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" Y: ");
    Serial.print(y);
    Serial.print(" Angle: ");
    Serial.print(dataPoints[i].angle);
    Serial.print(" Distance: ");
    Serial.println(dataPoints[i].reading);
  }
}

void PrintSweepXY()
{
  for(int i = 0; i<curDataPoints; i++)
  {
    double radians = GetRadiansFromDegrees(dataPoints[i].angle);
    double radius = dataPoints[i].reading;

    double x = radius*cos(radians);
    double y = radius*sin(radians);

    Serial.print(x);
    Serial.print((char)0x09);
    Serial.println(y);
  }
}

void FindLinesFromSweep(boolean OnRight)
{
  REAL xList[staticDataSet];
  REAL yList[staticDataSet];
  for(int i = 0; i < curDataPoints; i++)
  {
    double radians = GetRadiansFromDegrees(dataPoints[i].angle);
    double radius = dataPoints[i].reading;

    double x = radius*cos(radians);
    double y = radius*sin(radians);

    xList[i] = x;
    yList[i] = y;
  }

  FindBestFitLineInDataSet(xList, yList, curDataPoints, OnRight);
}

void Sweep(double fromAngle, double toAngle)
{
  Initialize();
  curDataPoints = 0;

  boolean done = false;

  while(!done)
  {
    stepForward();
    if(GetAngleFromStep(curStep) > fromAngle && GetAngleFromStep(curStep) < toAngle)
    {
      delay(100);
      Serial.print("Read at Angle: ");
      Serial.println(GetAngleFromStep(curStep));
      int distance = myLidarLite.distance();
      double Angle = GetAngleFromStep(curStep);

      LidarRead curRead;
      curRead.angle = Angle;
      curRead.reading = distance;
      dataPoints[curDataPoints] = curRead;
      curDataPoints++;
    }
    else if(GetAngleFromStep(curStep) > toAngle)
    {
      Serial.println("End Sweep");
      done = true;
    }
  }
}



int curWayPoint = 0;
boolean DataSent = false;
boolean sweeping = false;
void loop()
{
  if(curWayPoint == 1)
  {
    if(!DataSent)
    {
      FLUSHMOTORBUFFER();
      MoveMotorToAngle(75);
      DataSent=true;
      sweeping = false;
    }
  }
  else if(curWayPoint == 2)
  {
    if(!DataSent)
    {
      FLUSHMOTORBUFFER();
      MoveMotorForward(1);
      DataSent = true;
      sweeping = false;
    }
  }
  else if(curWayPoint == 3)
  {
    if(!DataSent)
    {
      sweepFrom = 100;
      sweepTo = 180;
      DataSent = true;
      sweeping = true;
    }

    Sweep(sweepFrom, sweepTo);
    FindLinesFromSweep(true);
  }
  else if(curWayPoint == 4)
  {
    if(!DataSent)
    {
      FLUSHMOTORBUFFER();
      MoveMotorToAngle(255);
      DataSent = true;
      sweeping = false;
    }
  }
  else if(curWayPoint == 5)
  {
    if(!DataSent)
    {
      FLUSHMOTORBUFFER();
      MoveMotorForward(1);
      DataSent = true;
      sweeping = false;
    }
  }
  
  // if(!SweepDone && !SweepError)
  // {
  //   Sweep(sweepFrom, sweepTo);
  //   //PrintSweepInfo();
  //   FindLinesFromSweep(true);
  //   PrintSweepXY();
  // }
  // else if(SweepError)
  // {
  //   sweepTo = 90;
  //   sweepFrom = 0;
  //   Serial.println("SWEEP ERROR");
  // }

  if(CheckForMotionComplete() && !sweeping)
  {
    // Move on to next motion
    curWayPoint++;
    DataSent = false;
    Serial.print("Moving to Waypoint ");
    Serial.println(curWayPoint);
  }
  else if(sweeping && SweepDone)
  {
    FLUSHMOTORBUFFER();
    curWayPoint++;
    DataSent = false;
    sweeping = false;
    Serial.print("Moving to Waypoint ");
    Serial.println(curWayPoint);
  }
}


void fullRotation(){
  FoundZero = true;
}


