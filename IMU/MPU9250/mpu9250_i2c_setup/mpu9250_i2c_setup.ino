/*
 * This code configures the MPU9250 IMU for I2C communication with the STM32 Bluepill
 */

#include <Wire.h>

const int MPU = 0x68;                      // I2C address of the MPU9250
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; // MPU values

void setup()
{
  Wire.begin();                // Initiate the library and join the I2C bus
  Wire.beginTransmission(MPU); // Begin a transmission with the given address
  Wire.write(0x6B);            // Send value in bytes
  Wire.write(0);
  Wire.endTransmission(true); // End the transmission
}

void loop()
{
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

  // Repeat for each value

  Serial.print(GyY);
  Serial.print(",");
  Serial.print(GyZ);
}
