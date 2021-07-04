/* This code reads [ACCELEROMETER, GYROSCOPE and MAGNETOMETER'S] raw data from the BNO055 IMU Sensor

   Connections
   ===========
   Connect SCL to SCL pin (analog 5 on Arduino UNO)
   Connect SDA to SDA pin (analog 4 on Arduino UNO)
   Connect Vin to 3-5V DC (depending on your board's logic level)
   Connect GROUND to common ground

*/

/* Units 

 - VECTOR_ACCELEROMETER - m/s^2
 - VECTOR_MAGNETOMETER  - uT
 - VECTOR_GYROSCOPE     - rad/s
 - VECTOR_EULER         - degrees
 - VECTOR_LINEARACCEL   - m/s^2
 - VECTOR_GRAVITY       - m/s^2
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
  Serial.print(mg, DEC);
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
  
  // Get temperature from the IMU and save it into a 8-bit int variable named 'temp'
  int8_t temp = IMU.getTemp();
  
  // Print the temperature through the Serial Monitor
  Serial.println(temp);
  
  // Change the time clock on the chip to the time clock on board of the IMU
  IMU.setExtCrystalUse(true);
}

void loop() {
  
  // Display Calibration status (This line is optional)
  // displayCalStatus();
    
  // Work with the imu sensor from Adafruit library
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'acc' (accelerometer) for the specific object IMU
  imu::Vector<3> acc = IMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'gyro' (gyroscope) for the specific object IMU
  imu::Vector<3> gyro = IMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Go to BNO055's 'imu' and bring back a vector of 3 components into 'mag' (magnetometer) for the specific object IMU
  imu::Vector<3> mag = IMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  // Print Acceleration data
  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());
  Serial.print(",");
  
  // Print Gyroscope data
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print(gyro.z());
  Serial.print(",");

  // Print Magnetometer data
  Serial.print(mag.x());
  Serial.print(",");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.println(mag.z());

  // Wait the specified delay before requesting next data
  delay(BNO055_SAMPLERATE_DELAY_MS);

}
