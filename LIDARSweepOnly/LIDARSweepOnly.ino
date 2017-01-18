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

#define REAL double
#define MAX_READS 300

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

int num_Readings = 0;

boolean FoundZero = false;

#define staticDataSet 300
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

void FindBestFitLineInDataSet(REAL x[], REAL y[], int n, int WallShouldBeOnRight)
{
  int acceptableNumberOfReads = 3;
  double acceptableRSquared = 0.95;
  double outlierThreshold = 30.0;
  int maxCycles = 5;
  REAL newX[MAX_READS];
  REAL newY[MAX_READS];

  REAL m,b,r;
  int lineStatus = linreg(n,x,y,&m,&b,&r);

  if(lineStatus == 1)
  {
    Serial.println("Perfectly Parallel");
  }
  else
  {
    double Angle = atan(m)*180/3.14;
    REAL rSqured = r*r;

    Serial.print("Angle: ");
    Serial.print(Angle);
    Serial.print(" rSquared: ");
    Serial.println(rSqured);

  //   int curCycle = 0;
  //   while(rSqured < acceptableRSquared && curCycle < maxCycles)
  //   {
  //     curCycle++;
  //     // Cut off all data on left side of line
  //     if(WallShouldBeOnRight)
  //     {
  //       int numAdded = 0;
  //       for(int i = 0; i < n; i++)
  //       {
  //         double expectedXforY = (y[i]-b)/m;
  //         if(expectedXforY <= x[i])
  //         {
  //           newX[numAdded] = x[i];
  //           newY[numAdded] = y[i];
  //           numAdded++;
  //       }

  //       n = numAdded;
  //     }
  //     else
  //     {
  //       int numAdded = 0;
  //       for(int i = 0; i < n; i++)
  //       {
  //         double expectedXforY = (y[i]-b)/m;
  //         if(expectedXforY >= x[i])
  //         {
  //           newX[numAdded] = x[i];
  //           newY[numAdded] = y[i];
  //           numAdded++;
  //         }
  //         else
  //         {
  //           cout << "Expected X:" << expectedXforY << " For ValY: " << y[i] << endl;
  //         }
  //       }

  //       n = numAdded;
  //     }

  //     for(int i = 0; i < n; i++)
  //     {
  //         x[i] = newX[i];
  //         y[i] = newY[i];
  //     }

  //   lineStatus = linreg(n,x,y,&m,&b,&r);
  //   rSqured = r*r;
  //   double Angle = atan(m)*180/3.14;
  //   cout << "LineStatus: " <<  lineStatus << endl;
  //   cout << "Slope: " << m <<  " SlopeAngle: " << Angle << " Intercept: " << b << " R^2:" << rSqured << endl;
  // }
  }
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

void loop()
{

  Sweep(40, 100);
  //PrintSweepInfo();
  FindLinesFromSweep(true);
  PrintSweepXY();
}


void fullRotation(){
  FoundZero = true;
}


