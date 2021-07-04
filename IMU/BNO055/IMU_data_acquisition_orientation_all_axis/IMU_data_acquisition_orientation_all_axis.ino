/* This code reads [ACCELEROMETER] and [GYROSCOPE] data from the BNO055 IMU Sensor
 * with a Complementary Filter for the output signal
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

float theta_gyro = 0, phi_gyro = 0;
float Dt; // Change in time
unsigned long millisOld; // Marker of t[n-1]

float x_magn, y_magn; // X and Y coordinates of the IMU's magnetometer
float psi; // Yaw angle
float theta_rad, phi_rad;

void setup() {
  // Set the baud rate speed. This is how fast the data is to be sent through the USB connection
  Serial.begin(115200);
  
  // Start the IMU sensor
  IMU.begin();
    
  // Wait 1000 ms to ensure the sensor starts correctly
  delay(1000);
  
  // Change the time clock on the chip to the time clock on board of the IMU
  IMU.setExtCrystalUse(true);

  // Start millis counter
  millisOld = millis();
}

void loop() {

  // Work with the imu sensor from Adafruit library
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'acc' (accelerometer) for the specific object IMU
  imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'gyro' (gyroscope) for the specific object IMU
  imu::Vector<3> gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'magn' (magnetometer ) for the specific object IMU
  imu::Vector<3> magn = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  // Parameters
  // Tilt approximation and normalize it to '1g' and convert into degrees
  theta_meas = - atan2(acc.x()/9.81,acc.z()/9.81)/(2*pi)*360;
  phi_meas = - atan2(acc.y()/9.81,acc.z()/9.81)/(2*pi)*360;

  theta_filt_new = theta_filt_old*0.9 + theta_meas*0.1;
  phi_filt_new = phi_filt_old*0.9 + phi_meas*0.1;

  // Change in time (seconds)
  Dt = (millis()-millisOld)/1000.;
  millisOld = millis();

  // Complementary filter
  theta = (theta + gyro.y()*Dt)*0.95 + theta_meas*0.05;
  phi = (phi - gyro.x()*Dt)*0.95 + phi_meas*0.05; // Minus sign is for the particular IMU orientation
  
  theta_gyro = theta_gyro + gyro.y()*Dt;
  phi_gyro = phi_gyro - gyro.x()*Dt; // Negative sign due to body axis

  // Yaw angle
  phi_rad = phi*2*pi/360;
  theta_rad = theta*2*pi/360;
  // Project vectors into X-Y plane
  x_magn = magn.x()*cos(theta_rad) - magn.y()*sin(phi_rad)*sin(theta_rad) + magn.z()*cos(phi_rad)*sin(theta_rad);
  y_magn = magn.y()*cos(phi_rad) + magn.z()*sin(phi_rad);
  psi = atan2(y_magn,x_magn)/(2*pi)*360;
  
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
  Serial.print(phi_filt_new);

  // Gryoscope data
  Serial.print(",");
  Serial.print(gyro.y()); // Pitch Rotational velocity
  Serial.print(",");
  Serial.print(gyro.x()); // Roll Rotational velocity

  Serial.print(",");
  Serial.print(theta_gyro); // Pitch
  Serial.print(",");
  Serial.print(phi_gyro); // Roll

  // Complementary filter data
  Serial.print(",");
  Serial.print(theta); // Pitch
  Serial.print(",");
  Serial.print(phi); // Roll

  // Complementary filter data
  Serial.print(",");
  Serial.println(psi); // Psi

  
  // Next iteration
  theta_filt_old = theta_filt_new;
  phi_filt_old = phi_filt_new;
  
  // Wait the specified delay before requesting next data
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
