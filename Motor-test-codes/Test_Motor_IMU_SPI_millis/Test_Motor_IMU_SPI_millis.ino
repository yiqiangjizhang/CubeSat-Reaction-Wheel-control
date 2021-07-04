/*
  * ---------- TEST IMU IN SPI + MOTOR IN I2C + millis
  * 
  * The following code reads values from the IMU and performs an acceleration and a deceleration of the motor (with a wating phase)
  * This test has been implemented with the IMU in SPI communication protocol.
  * 
  * Definitions established made to adapt to STM32 ports instead of Arduino (Library is designed for Arduino)
  * 1MHz of clock because SPI connection with IMU is max 1MHz (by datasheet)
  * 
  * 
  * SCMD library by SparkFun used (https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library)
  * MPU9250 library by Brian Chen used (https://github.com/brianc118/MPU9250)
  * Rest of libraries from Arduino
  * STM32 package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
  * 
  * 
  */

#include <Arduino.h>
#include <stdint.h>
#include "SCMD.h"
#include "SCMD_config.h" // Contains #defines for common SCMD register names and values
#include "Wire.h"
#include <SPI.h>
#include <MPU9250.h>

#define LEDPIN PC13       // STM32 LED pin
#define SPI_CLOCK 1000000 //  1MHz clock works.
#define CS1 PA4           // STM32 NCS pin
// MOSI on PA7, MISO  on PA6 and CLK on PA5 by default on STM32

#define WAITFORINPUT() {
while (!Serial.available())
{
};
while (Serial.available())
{
  Serial.read();
};
}

MPU9250 mpu(SPI_CLOCK, CS1); // MPU object
SCMD DriverOne;              // Driver object

unsigned long prevMillis = 0; // Keeps track of previous MIllis() value
unsigned long currMillis = 0; // Keeps track of current MIllis() value

int counter = 0;       // Counter for speed
bool laststate = true; // Current/Latest state of motor (false=decceleration, true=acceleration)
bool waiting = false;  // Check if the motor is at a const speed

int maxspeed = 255;   // Max velocity to reach
int delayramp = 100;  // Delay for increase in speed, in milisec
int delaywait = 5000; // Delay on waiting

void setup()
{

  counter = 0;             // Reset of counter
  pinMode(LEDPIN, OUTPUT); // LED definition
  pinMode(CS1, OUTPUT);    // Pin definitions
  digitalWrite(LEDPIN, HIGH);
  Serial.begin(9600); // Serial Monitor initiation
  Serial.println("Start");
  SPI.begin();

  Serial.println("Press any key to continue");
  WAITFORINPUT();

  // IMU
  mpu.init(true);

  uint8_t wai = mpu.whoami(); // Connection with mpu
  if (wai == 0x71)
  {
    Serial.println("Successful connection");
  }
  else
  {
    Serial.print("Failed connection: ");
    Serial.println(wai, HEX);
  }

  uint8_t wai_AK8963 = mpu.AK8963_whoami(); // Connection with mag
  if (wai_AK8963 == 0x48)
  {
    Serial.println("Successful connection to mag");
  }
  else
  {
    Serial.print("Failed connection to mag: ");
    Serial.println(wai_AK8963, HEX);
  }

  mpu.calib_acc();
  mpu.calib_mag();

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

  Serial.println("Send any char to begin main loop.");
  WAITFORINPUT();
}

void loop()
{

  // Rollover is done in the ifs, with currMilils-prevMIllis. if it reaches rollover the difference remains the sameas it is an unsigned value
  currMillis = millis();

  if (unsigned int(currMillis - prevMillis) >= delayramp && waiting == false)
  { // State of accel or decel. Accounts for millis() override
    prevMillis = currMillis;
    if (counter <= maxspeed && laststate == true)
    { // Acceleration
      counter = counter + 1;
      if (counter > maxspeed)
      { // Max speed reach
        waiting = true;
        counter = maxspeed;
      }
    }
    else if (counter >= 0 && laststate == false)
    { // Deceleration
      counter = counter - 1;
      if (counter < 0)
      { // 0 speed reach
        waiting = true;
        counter = 0;
      }
    }
  }

  if (unsigned int(currMillis - prevMillis) >= delaywait && waiting == true)
  { // State of waiting. Accounts for millis() override
    prevMillis = currMillis;
    waiting = false;
    if (laststate)
    { // Start of decel
      laststate = false;
    }
    else
    {
      laststate = true; // Start of accel
    }
  }

  DriverOne.setDrive(0, 0, counter);
  mpu.read_all();

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
  Serial.println(mpu.temperature);

  Serial.print("Millis: ");
  Serial.print(currMillis);
  Serial.print(", State: ");
  if (waiting == false)
  {
    if (laststate == true)
    {
      Serial.print("ACCELERATION");
    }
    else
    {
      Serial.print("DECELERATION");
    }
  }
  else
  {
    Serial.print("WAITING");
  }
  Serial.print(", Speed: ");
  Serial.println(counter);
  delay(10);
}
