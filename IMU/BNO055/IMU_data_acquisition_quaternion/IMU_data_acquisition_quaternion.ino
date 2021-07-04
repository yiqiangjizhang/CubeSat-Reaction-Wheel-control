/* This code reads [QUATERNIONS] data from the BNO055 IMU Sensor
*/

// Libraries
#include <Wire.h> // This library allows to communicate with I2C / TWI devices. 
#include <Adafruit_Sensor.h> // Library with drivers that are based on the Adafruit Unified Sensor Driver
#include <Adafruit_BNO055.h> // This is a library for the BNO055 orientation sensor
#include <utility/imumaths.h> // Inertial Measurement Unit Maths Library (it includes matrix.h, quaternions.h and vector.h)
#include <math.h> // Math functions

// Global parameters and objects
#define BNO055_SAMPLERATE_DELAY_MS (100) // Define how fast the sensor sample rate (sample every 100 ms)

Adafruit_BNO055 IMU = Adafruit_BNO055(); // Create IMU object and set what the object is


void setup() {
  // Set the baud rate speed. This is how fast the data is to be sent through the USB connection
  Serial.begin(115200);
  
  // Start the IMU sensor
  IMU.begin();
    
  // Wait 1000 ms to ensure the sensor starts correctly
  delay(1000);
  
  // Change the time clock on the chip to the time clock on board of the IMU
  IMU.setExtCrystalUse(true);

}

void loop() {

  // Work with the imu sensor from Adafruit library
  // Go to BNO055's 'imu' and bring back a vector of 4 components into 'quat' (quaternion) for the specific object IMU
  imu::Quaternion quat=IMU.getQuat();

  // Print Quaternion data
  Serial.print(quat.w()); // Real part
  Serial.print(",");
  Serial.print(quat.x()); // 'i'
  Serial.print(",");
  Serial.print(quat.y()); // 'j'
  Serial.print(",");
  Serial.println(quat.z()); // 'k'
  
  // Wait the specified delay before requesting next data
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
