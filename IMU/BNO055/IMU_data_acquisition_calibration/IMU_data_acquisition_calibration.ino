/* This code reads calibrates the BNO055 IMU Sensor
*/

// Libraries
#include <Wire.h> // This library allows to communicate with I2C / TWI devices. 
#include <Adafruit_Sensor.h> // Library with drivers that are based on the Adafruit Unified Sensor Driver
#include <Adafruit_BNO055.h> // This is a library for the BNO055 orientation sensor
#include <utility/imumaths.h> // Inertial Measurement Unit Maths Library (it includes matrix.h, quaternions.h and vector.h)

// Global parameters and objects
#define BNO055_SAMPLERATE_DELAY_MS (100) // Define how fast the sensor sample rate (sample every 100 ms)

Adafruit_BNO055 IMU = Adafruit_BNO055(); // Create IMU object and set what the object is

// Calibration function
void displayCalStatus(void)
{
  /*  Get the four calibration values (0..3) 
      Any sensor data reporting 0 should be ignored, 
      3 means 'fully calibrated" */
  uint8_t system, gyros, accel, mg =0;
  IMU.getCalibration(&system, &gyros, &accel, &mg);

  // The data should be ignored until the system calibration is > 0
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  // Display the individual values
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" Gyros:");
  Serial.print(gyros, DEC);
  Serial.print(" Accel:");
  Serial.print(accel, DEC);
  Serial.print(" Magne:");
  Serial.println(mg, DEC);
}

void setup() {
  // Set the baud rate speed. This is how fast the data is to be sent through the USB connection
  Serial.begin(115200);
  
  // Print initial message
  Serial.println("BNO055 IMU Sensor Raw Data Initialized"); 
  Serial.println("");
  
  // Start the IMU sensor
  IMU.begin();
    
  // Wait 1000 ms to ensure the sensor starts correctly
  delay(1000);
  
  // Change the time clock on the chip to the time clock on board of the IMU
  IMU.setExtCrystalUse(true);
}

void loop() {
  
  displayCalStatus();
    
  // Wait the specified delay before requesting next data
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
