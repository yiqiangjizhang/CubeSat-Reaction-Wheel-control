/*
  * 
  * ---------- TEST SMT32 1 MOTOR + IMU, SAME I2C PORT (MOTOR IN I2C, IMU IN I2C2)
  * The following code reads values from the IMU and performs a linear ramp to the maximum power of the motor, then decreases linearly as well.
  * This test has been implemented with I2C communication protocol.
  * 
  * 
  * SCMD library by SparkFun used (https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library)
  * Rest of libraries from Arduino
  * STM32 package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
  * 
  */

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" // Contains #defines for common SCMD register names and values
#include <Wire.h>

#define LEDPIN PC13 // STM32 LED pin

const int MPU = 0x68;                      // I2C address of the MPU9250
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // MPU values

SCMD DriverOne; // Driver object

void setup()
{

  Wire.begin();            // Initiate the library and join the I2C bus
  Serial.begin(9600);      // Data rate in bytes per second   // Serial Monitor initiation
  pinMode(LEDPIN, OUTPUT); // LED definition
  Serial.println("Start");

  // IMU
  Wire.beginTransmission(MPU); // Begin a transmission with the given address
  Wire.write(0x6B);            // Send value in bytes
  Wire.write(0);
  Wire.endTransmission(true); // End the transmission

  // MOTOR
  DriverOne.settings.commInterface = I2C_MODE; // Driver mode definition
  DriverOne.settings.I2CAddress = 0x5D;        // Driver address definition (0x5D by default)

  while (DriverOne.begin() != 0xA9)
  { // Wait for idle
    Serial.println("ID Mismatch");
    delay(200);
  }
  Serial.println("ID Match");

  Serial.println("Waiting for enumeration"); // Wait for peripherals (enumeration)
  while (DriverOne.ready() == false)
    ;
  Serial.println("Ready");

  while (DriverOne.busy())
    ; // Enables the driver
  DriverOne.enable();
}

void loop()
{

  int vel = 255;
  int steps = 20;

  for (int i = 0; i <= vel; i++)
  {

    DriverOne.setDrive(0, 0, i);
    Wire.beginTransmission(MPU); // Begin a transmission with the given address
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 12, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    Serial.print("IMU values: ");
    Serial.print(AcX);
    Serial.print(",");
    Serial.print(AcY);
    Serial.print(",");
    Serial.print(AcZ);
    Serial.print(",");
    Serial.print(GyX);
    Serial.print(",");
    Serial.print(GyY);
    Serial.print(",");
    Serial.print(GyZ);
    Serial.print(",   Speed: ");
    Serial.println(i);
    delay(steps);
  }
  delay(5000);

  for (int i = vel; i >= 0; i--)
  {

    DriverOne.setDrive(0, 0, i);
    Wire.beginTransmission(MPU); // Begin a transmission with the given address
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 12, true);

    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    GyX = Wire.read() << 8 | Wire.read();
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    Serial.print("IMU values: ");
    Serial.print(AcX);
    Serial.print(",");
    Serial.print(AcY);
    Serial.print(",");
    Serial.print(AcZ);
    Serial.print(",");
    Serial.print(GyX);
    Serial.print(",");
    Serial.print(GyY);
    Serial.print(",");
    Serial.print(GyZ);
    Serial.print(",   Speed: ");
    Serial.println(i);
    delay(steps);
  }
  delay(5000);
}
