/* 
 * This code configures the MPU9250 IMU for SPI communication with the STM32 Bluepill and prints data.
 *
 * MPU9250 library by Brian Chen used (https://github.com/brianc118/MPU9250)
*/

#include "Wire.h"
#include <SPI.h>
#include <MPU9250.h>

#define SPI_CLOCK 1000000 // 1MHz clock works.
#define CS1 PA4           // STM32 NCS pin
#define WAITFORINPUT() {  // Function to read serial imputs
while (!Serial.available())
{
};
while (Serial.available())
{
  Serial.read();
};
}

MPU9250 mpu(SPI_CLOCK, CS1); // MPU object

void setup()
{
  pinMode(CS1, OUTPUT); // Pin definition
  SPI.begin();          // Initiate the library and join the SPI bus

  Serial.println("Press any key to continue");
  WAITFORINPUT();

  mpu.init(true); // Initiate the IMU

  uint8_t wai = mpu.whoami(); // Connection status with mpu
  if (wai == 0x71)
  {
    Serial.println("Successful connection");
  }
  else
  {
    Serial.print("Failed connection: ");
    Serial.println(wai, HEX);
  }

  uint8_t wai_AK8963 = mpu.AK8963_whoami(); // Connection status with mag
  if (wai_AK8963 == 0x48)
  {
    Serial.println("Successful connection to mag");
  }
  else
  {
    Serial.print("Failed connection to mag: ");
    Serial.println(wai_AK8963, HEX);
  }

  mpu.calib_acc(); // Calibrations
  mpu.calib_mag();

  Serial.println("Send any char to begin main loop.");
  WAITFORINPUT();
}

void loop()
{
  mpu.read_all();
  // Data is stored in variables from the library
  Serial.print(mpu.gyro_data[0]);
  Serial.print('\t');
  Serial.print(mpu.gyro_data[1]);
  Serial.print('\t');
  Serial.print(mpu.gyro_data[2]);
  Serial.print('\t');
  Serial.print(mpu.accel_data[0]);
  Serial.print('\t');
  Serial.print(mpu.accel_data[1]);
  Serial.print('\t');
  Serial.print(mpu.accel_data[2]);
  Serial.print('\t');
  Serial.print(mpu.mag_data[0]);
  Serial.print('\t');
  Serial.print(mpu.mag_data[1]);
  Serial.print('\t');
  Serial.print(mpu.mag_data[2]);
  Serial.print('\t');
  Serial.print(mpu.temperature);
}
