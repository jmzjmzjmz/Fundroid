#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Calibration done in New Jersey

Fully calibrated!
--------------------------------
Calibration Results: 
Accelerometer: 20 65454 0 
Gyro: 0 65534 65535 
Mag: 65354 221 65323 
Accel Radius: 1000
Mag Radius: 839

Storing calibration data to EEPROM...
Data stored to EEPROM.

--------------------------------
*/

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

void setup(void)
{
  Serial.begin(9600);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  adafruit_bno055_offsets_t calibrationData;
  SetCalibrationData(calibrationData);
  bno.setSensorOffsets(calibrationData);

  bno.setExtCrystalUse(true);

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
}

void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("     CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}