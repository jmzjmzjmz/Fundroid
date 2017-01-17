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

LIDARLite myLidarLite;

int dirPin = 2;
int stepPin = 3;
int stepTime = 2;
int curStep = 0;

int optoPin = 11;
double angleOffset = 0;

int num_Readings = 0;

boolean FoundZero = false;

#define staticDataSet 5000
#define StepsPerRotation 200

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

void setup()
{
  delay(1000);
  
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
      done = true;
    }
  }
}

void loop()
{

  

}


void fullRotation(){
  FoundZero = true;
}


