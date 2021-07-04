/* This code reads all sensors data from the BNO055 IMU Sensor
 *  and uses bluetooth communication
*/

// Libraries
#include <Wire.h>             // This library allows to communicate with I2C / TWI devices.
#include <Adafruit_Sensor.h>  // Library with drivers that are based on the Adafruit Unified Sensor Driver
#include <Adafruit_BNO055.h>  // This is a library for the BNO055 orientation sensor
#include <utility/imumaths.h> // Inertial Measurement Unit Maths Library (it includes matrix.h, quaternions.h and vector.h)
#include <math.h>             // Math functions
#include <SoftwareSerial.h>   // This library enables bluetooth communcication

// Global parameters and objects
#define BNO055_SAMPLERATE_DELAY_MS (100) // Define how fast the sensor sample rate (sample every 100 ms)

Adafruit_BNO055 IMU = Adafruit_BNO055(); // Create IMU object and set what the object is

const int Rx = 2;            // Connect Arduino Digital Pin 2 to HC06 Pin Tx
const int Tx = 3;            // Connect Arduino Digital Pin 3 to HC06 Pin Rx
SoftwareSerial hc06(Rx, Tx); // Set Receive and Transmission pins

/* Function Prototypes*/
// Calibration function
void displayCalStatusSerial(void);
void displayCalStatusBluetooth(void);
void printSerial(void);
void prinBluetooth(void);

void setup()
{
  //Initialize Serial Monitor
  Serial.begin(9600);

  // Start the IMU sensor
  IMU.begin();

  //Initialize Bluetooth Serial Port
  hc06.begin(9600);

  // Wait 1000 ms to ensure the sensor starts correctly
  delay(1000);

  // Change the time clock on the chip to the time clock on board of the IMU
  IMU.setExtCrystalUse(true);
}

void loop()
{

  // Print calibration data through serial
  // displayCalStatusSerial();
  // Print calibration data
  displayCalStatusBluetooth();

  // Print data over Bluetooth port
  printBluetooth();
  // Print data over Serial Port
  // printSerial();

  //Write data from HC06 to Serial Monitor
  if (hc06.available())
  {
    Serial.write(hc06.read());
  }

  //Write from Serial Monitor to HC06
  if (Serial.available())
  {
    hc06.write(Serial.read());
  }

  // Wait the specified delay before requesting next data
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

// Calibration function
void displayCalStatusSerial(void)
{
  /*  Get the four calibration values (0..3) 
      Any sensor data reporting 0 should be ignored, 
      3 means 'fully calibrated" */
  uint8_t system, gyros, accel, mg = 0;
  IMU.getCalibration(&system, &gyros, &accel, &mg);

  // The data should be ignored until the system calibration is > 0
  // Display the individual values
  Serial.print(system, DEC);
  Serial.print(",");
  Serial.print(gyros, DEC);
  Serial.print(",");
  Serial.print(accel, DEC);
  Serial.print(",");
  Serial.print(mg, DEC);
}

// Calibration function for bluetooth
void displayCalStatusBluetooth(void)
{
  /*  Get the four calibration values (0..3) 
      Any sensor data reporting 0 should be ignored, 
      3 means 'fully calibrated" */
  uint8_t system, gyros, accel, mg = 0;
  IMU.getCalibration(&system, &gyros, &accel, &mg);

  // The data should be ignored until the system calibration is > 0
  // Display the individual values
  hc06.print(system, DEC);
  hc06.print(",");
  hc06.print(gyros, DEC);
  hc06.print(",");
  hc06.print(accel, DEC);
  hc06.print(",");
  hc06.print(mg, DEC);
}

// Print serial
void printSerial(void)
{
  // Work with the imu sensor from Adafruit library
  // Go to BNO055's 'imu' and bring back a vector of 4 components into 'quat' (quaternion) for the specific object IMU
  imu::Quaternion quat = IMU.getQuat();
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'acc' (accelerometer) for the specific object IMU
  imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'gyro' (gyroscope) for the specific object IMU
  imu::Vector<3> gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'magn' (magnetometer ) for the specific object IMU
  imu::Vector<3> magn = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Print Quaternion data
  Serial.print(quat.w()); // Real part
  Serial.print(",");
  Serial.print(quat.x()); // 'i'
  Serial.print(",");
  Serial.print(quat.y()); // 'j'
  Serial.print(",");
  Serial.print(quat.z()); // 'k'
  Serial.print(",");

  // Print Acceleration data
  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());
  Serial.print(",");

  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.x());
  Serial.print(",");

  Serial.print(magn.x());
  Serial.print(",");
  Serial.print(magn.y());
  Serial.print(",");
  Serial.println(magn.z());
}

// Print Bluetooth
void printBluetooth(void)
{
  // Work with the imu sensor from Adafruit library
  // Go to BNO055's 'imu' and bring back a vector of 4 components into 'quat' (quaternion) for the specific object IMU
  imu::Quaternion quat = IMU.getQuat();
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'acc' (accelerometer) for the specific object IMU
  imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'gyro' (gyroscope) for the specific object IMU
  imu::Vector<3> gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'magn' (magnetometer ) for the specific object IMU
  imu::Vector<3> magn = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  // Print Quaternion data
  hc06.print(quat.w()); // Real part
  hc06.print(",");
  hc06.print(quat.x()); // 'i'
  hc06.print(",");
  hc06.print(quat.y()); // 'j'
  hc06.print(",");
  hc06.print(quat.z()); // 'k'
  hc06.print(",");

  // Print Acceleration data
  hc06.print(acc.x());
  hc06.print(",");
  hc06.print(acc.y());
  hc06.print(",");
  hc06.print(acc.z());
  hc06.print(",");

  hc06.print(gyro.x());
  hc06.print(",");
  hc06.print(gyro.x());
  hc06.print(",");
  hc06.print(gyro.x());
  hc06.print(",");

  hc06.print(magn.x());
  hc06.print(",");
  hc06.print(magn.y());
  hc06.print(",");
  hc06.println(magn.z());
}
