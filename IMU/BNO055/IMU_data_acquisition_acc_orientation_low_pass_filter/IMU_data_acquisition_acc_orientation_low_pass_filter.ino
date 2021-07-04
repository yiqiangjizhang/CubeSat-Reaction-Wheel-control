/* This code reads [ACCELEROMETER] from the BNO055 IMU Sensor and calculates pitch and roll angles
 *  with an additional Low Pass Filter
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

const float pi = 3.1415926535; // Number Pi

//Variables
float theta, phi;
float theta_meas, phi_meas;
float theta_filt_old = 0, phi_filt_old = 0;
float theta_filt_new, phi_filt_new;

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

  // Work with the imu sensor from Adafruit library
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'acc' (accelerometer) for the specific object IMU
  imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Parameters
  // Tilt approximation and normalize it to '1g' and convert into degrees
  theta_meas = - atan2(acc.x()/9.81,acc.z()/9.81)/(2*pi)*360;
  phi_meas = - atan2(acc.y()/9.81,acc.z()/9.81)/(2*pi)*360;

  theta_filt_new = theta_filt_old*0.9 + theta_meas*0.1;
  phi_filt_new = phi_filt_old*0.9 + phi_meas*0.1;

  // Print Acceleration data
  Serial.print(acc.x()/9.81);
  Serial.print(",");
  Serial.print(acc.y()/9.81);
  Serial.print(",");
  Serial.print(acc.z()/9.81);
  Serial.print(",");
  // Raw data
  Serial.print(theta_meas);
  Serial.print(",");
  Serial.print(phi_meas);
  Serial.print(",");
  // Filtered data
  Serial.print(theta_filt_new);
  Serial.print(",");
  Serial.println(phi_filt_new);

  // Next iteration
  theta_filt_old = theta_filt_new;
  phi_filt_old = phi_filt_new;
  
  // Wait the specified delay before requesting next data
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
