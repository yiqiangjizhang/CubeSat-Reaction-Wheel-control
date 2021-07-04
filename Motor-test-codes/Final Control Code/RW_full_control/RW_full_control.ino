/* Reaction Wheel controller
 *
 * The following code provides a control for PLATHON project's CubeSat.
 *
 * 
 * The following libraries are used:
 * STM32 Bluepill microcontroller package from http://dan.drown.org/stm32duino/package_STM32duino_index.json
 * SCMD Driver library by SparkFun used (https:// github.com/sparkfun/SparkFun_Serial1_Controlled_Motor_Driver_Arduino_Library)
 * MPU9250 IMU library by Rafa Castalla used (https://github.com/rafacastalla/MPU9250-1)
 * read_IMU() and IMU setup content code extracted from Andres Gomez and Miquel Reurer, from the PLATHON group (magnetorquers section)
 */

// ██████████████████████████████████████████████████████████████████████ DEFINITIONS
// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ LIBRARIES
#include <Arduino.h>     // Arduino library
#include <stdint.h>      // Integer types library
#include <SCMD.h>        // Serial Controlled Motor Driver library
#include <SCMD_config.h> // Serial Controlled Motor Driver Configuration library
#include <SPI.h>         // SPI library
#include <MPU9250.h>     // IMU library
#include <Wire.h>

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ DEFINITIONS
#define LEDPIN PC13                // Integrated LED of the Bluepill
#define CS1 4                      // STM32 NCS Chip select (SPI mode only) pin. MOSI on PA7, MISO on PA6 and CLK on PA5 by default on STM32
#define STM32_CLOCK 72000          // Internal STM32 Clock (72MHz, in kHz so period is millisec)
#define PI 3.14159265              // Number PI
#define Rad_to_deg 57.29577951     // Convert radians to degrees
#define Deg_to_rad 0.01745329      // Convert degrees to radians
#define Final_pointing_tolerance 1 // Tolerance for final pointing (+-1 degree of tolerance) (Check if position is within the margin)
#define Pointing_mode_tolerance 5  // Tolerance for selecting pointing mode (+-5 degree of tolerance) (Select whether to use coarse or fine mode)
#define Accel_tolerance 0.5        // Tolerance for acceleration measurement (+-0.5 unit of acceleration of tolerance) (Acceptable error of acceleration)
#define Gyro_tolerance 1           // Tolerance for gyro measurement (+-1 unit of gyroscope Z tolerance) (Accounts to know if it is rotating or not at a constant speed for the ramp)

SCMD DriverOne;        // Driver Object definition
MPU9250 IMU(SPI, CS1); // MPU object definition

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GLOBAL VARIABLES
float IMU_accel_data_X, IMU_accel_data_Y, IMU_accel_data_Z; // Values of local IMU accelerometer (m/s^2)
float IMU_gyro_data_X, IMU_gyro_data_Y, IMU_gyro_data_Z;    // Values of local IMU gyroscope (degrees/s)
float IMU_mag_data_X, IMU_mag_data_Y, IMU_mag_data_Z;       // Values of local IMU magnetometer (uT)
float Pitch_deg, Roll_deg, Yaw_deg = 0;                     // Pitch, Roll and Yaw (degrees)

// Declare variables outside IMU, since variables initiates to 0 without defining them in function
float Accel_total_vector_modulus;       // Accelerometer total vector magnitude (m/s^2)
float Accel_pitch_deg, Accel_roll_deg;  // Accelerometer pitch and roll angles (degrees)
float Gyro_pitch_deg, Gyro_roll_deg;    // Gyroscope pitch and roll angles (degrees)
bool Gyro_Accel_sync = false;           // Check if Gyroscope and Accelerometer are synchronized
float Mag_X_calc, Mag_Y_calc;           // Calculated Magnetometer value (uT)
float Mag_X_corrected, Mag_Y_corrected; // Corrected value Magnetometer value (with damping) (uT)
float Pitch_rad, Roll_rad;              // Pitch and Roll angles (rad)

int RW_speed = 0;       // Value of Reaction Wheel speed [from -255 to 255]
int OBC_mode_value = 0; // Value of On Board Computer mode (waiting, positioning, detumbling, etc.)
int OBC_data_value = 0; // Value of On Board Computer data (desired angle turn, etc.)
int Stop_state = 0;     // If Stop_state != 0, motor stops rotating

// PID control definitions
double PID_error, PID_last_error;            // Initialize error and previousError
double PID_cumulative_error, PID_rate_error; // Initialize the cumulative Error (Integral) and the rate of Error (Derivative)
float Current_Yaw_deg = 0;                   // Addition of 180 deg to all values. The error is a substract, so the difference is the same

double kp = 3; // Proportional contribution
double ki = 4; // Integral contribution
double kd = 0; // Derivative contribution

float Deg_to_reach = 0;  // Setpoint angle (degrees)
bool Zero_state = false; // Check if PID encompass yaw of 0 degrees (to turn the value to a workable zone)
int PID_output = 0;      // Output value of the PID [from 0 to 255]

// ██████████████████████████████████████████████████████████████████████ FUNCTIONS
// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ GENERAL USE FUNCTIONS
// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ OBC Functions
void OBC_mode_receive()
{
  /*
    Function to set the Bluepill to receiving mode. Uses Timer CH3. Serial1 as UART communication

    INPUT: None, but gets mode from User Serial1 input
    OUTPUT: None, but saves OBC_mode_value
  */

  // Check if UART Serial1 is available
  if (Serial1.available() > 0)
  {
    String bufferString = ""; // String for buffer of Serial1
    // Keep saving input prompt
    while (Serial1.available() > 0)
    {
      bufferString += (char)Serial1.read(); // Adds chars to the Serial1 buffer
    }
    // Conversion from String to int
    OBC_mode_value = bufferString.toInt();
    Serial1.print("Mode Number: ");
    Serial1.println(OBC_mode_value);
  }
}

void OBC_data_receive()
{
  /*
    Function to set the Bluepill to receiving data mode. Uses Timer CH3. Serial1 as UART communication

    INPUT: None, but gets data from User Serial1 input
    OUTPUT: Save OBC_data_value
  */

  // Check if UART Serial1 is available
  if (Serial1.available() > 0)
  {
    String bufferString = ""; // String for buffer of Serial1
    // Keep saving input prompt
    while (Serial1.available() > 0)
    {
      bufferString += (char)Serial1.read(); // adds chars to the Serial1 buffer
    }
    OBC_data_value = bufferString.toInt(); // Conversion from String to int
  }
}

void EmergencyStop()
{
  /*
    Function to STOP the motor from rotating.

    INPUT: None, but gets data from User Serial1 input
    OUTPUT: Save OBC_data_value
  */

  // Check if UART Serial1 is available
  if (Serial1.available() > 0)
  {
    String bufferString = ""; // String for buffer of Serial1
    // Keep saving input prompt
    while (Serial1.available() > 0)
    {
      bufferString += (char)Serial1.read(); // adds chars to the Serial1 buffer
    }
    Stop_state = bufferString.toInt();         // Conversion from String to int
    Serial1.print("Emergency Stop activated"); // In this case turn, but can be what is needed
  }

  //  Timer1.attachInterrupt(TIMER_CH4, read_show_IMU);
  // read_show_IMU();
  // mode_Select(OBC_mode_value);

  if (Stop_state != 0)
  {
    // If it stopped
    set_impulse(0, 0);
    OBC_data_value = 0;
    mode_OBC_Input_Wait();
  }
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Functions
void set_impulse(bool RW_direction, int New_RW_speed)
{
  /*
    Function to set the motor at a fixed direction and speed (B = port 1, A = port 0)

    INPUT: 
      RW_direction: Reaction Wheel direction [Clockwise or Counter Clockwise] in A
      New_RW_speed: New reaction wheel speed [from 0 to 255]
    OUTPUT: 
      None, but saves speed of the Reaction Wheel
  */

  // Account for change in direction
  // If motor's z-axis direction is coincident with CubeSat's z-axis direction
  /**
   * RW_direction == true  ---> CounterClockwise, motor direction = 0
   * RW_direction == false  ---> Clockwise, motor direction = 1
   * 
   * */
  // If motor's z-axis direction is CONTRARY to CubeSat's z-axis direction
  /**
   * RW_direction == true  ---> CounterClockwise, motor direction = 1
   * RW_direction == false  ---> Clockwise, motor direction = 0
   * 
   * */

  if (RW_direction) // Reaction Wheel Counter Clock Wise
  {
    DriverOne.setDrive(1, 0, New_RW_speed); // Change direction depending on motor connection
    RW_speed = New_RW_speed;
  }
  else // Reaction Wheel Clock Wise
  {
    DriverOne.setDrive(1, 1, New_RW_speed);
    RW_speed = -New_RW_speed;
  }
}

void generate_ramp(bool RW_direction, int Acc_ramp_time_duration, int RW_ramp_speed_reach, bool Acc_Dec_state)
{
  /*
    Function to generate ramps. In this case it is a Single Impulse.
    As we dont know the time it lasts, we have to see if speed changes on the cubesat. That is the use of the while.

    INPUT: 
      RW_direction: Reaction Wheel direction [Clockwise or Counter Clockwise]
      Acc_ramp_time_duration: Duration of ramp
      RW_ramp_speed_reach: Final speed reached
      Acc_Dec_state: Check wether in acceleration state ('1') or deceleration state ('0')
    OUTPUT: 
      None
  */

  Serial1.println("Ramp Start");
  bool waiting = true;

  set_impulse(RW_direction, RW_ramp_speed_reach);

  Timer1.attachInterrupt(TIMER_CH4, read_show_IMU);
  waiting = true;

  while (waiting)
  { // Range of tolerance
    if (Acc_Dec_state)
    { // If accelerating
      if (abs(IMU_gyro_data_Z) > Gyro_tolerance)
      { // If gyro is not 0 or so, means it has constant speed of rotation
        if (abs(IMU_accel_data_X) < Accel_tolerance)
        { // Stop of acceleration
          waiting = false;
        }
      }
    }
    else
    { // If decelerating
      if (abs(IMU_gyro_data_Z) < Gyro_tolerance)
      { // If gyro is not 0 or so, means it has constant speed of rotation
        if (abs(IMU_accel_data_X) < Accel_tolerance)
        { // Stop of acceleration
          waiting = false;
        }
      }
    }
    delay(1); // Delay for ensuring getting inside the while loop
  }
  waiting = true;
  Timer1.detachInterrupt(TIMER_CH4);
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Functions
void read_IMU()
{
  /*
    Function to read IMU_Data (and refreshes the global variables for IMU data).

    INPUT: 
    None
    OUTPUT: 
    None, but saves IMU variables and Roll, Pitch and Yaw
  */

  // Begin reading IMU and pitch, yaw, roll
  IMU.readSensor();

  // Accelerations
  IMU_accel_data_X = IMU.getAccelX_mss();
  IMU_accel_data_Y = IMU.getAccelY_mss();
  IMU_accel_data_Z = IMU.getAccelZ_mss();
  // Gyroscope
  IMU_gyro_data_X = (IMU.getGyroX_rads() * Rad_to_deg);
  IMU_gyro_data_Y = (IMU.getGyroY_rads() * Rad_to_deg);
  IMU_gyro_data_Z = (IMU.getGyroZ_rads() * Rad_to_deg);
  //Magnetometer
  IMU_mag_data_X = IMU.getMagX_uT();
  IMU_mag_data_Y = IMU.getMagY_uT();
  IMU_mag_data_Z = IMU.getMagZ_uT();

  // GetTime of IRS, in case value is correct if timer definition changes.
  // The dT is the time step of each interrupt, thus, each read of the IMU
  // First division of clock by preescaler (frequency of timer), then inversion (period of timer), then multiplication by overflow (period of channel)
  float dT = ((1 / ((float)STM32_CLOCK / (float)Timer1.getPrescaleFactor())) * Timer1.getOverflow()) * 0.001; // dT in seconds

  // Get the angle Pitch and Roll angle(w = w0 + dT*angular_vel)
  Gyro_pitch_deg += dT * (IMU.getGyroY_rads()) * Rad_to_deg;
  Gyro_roll_deg += dT * (IMU.getGyroX_rads()) * Rad_to_deg;

  // Gimbal lock compensation
  Gyro_pitch_deg = Gyro_pitch_deg + Gyro_roll_deg * sin(dT * (IMU.getGyroZ_rads()));
  Gyro_roll_deg = Gyro_roll_deg - Gyro_pitch_deg * sin(dT * (IMU.getGyroZ_rads()));

  // Modulus of the acceleration vector, calculate the total (3D) vector
  Accel_total_vector_modulus = sqrt((IMU.getAccelX_mss() * IMU.getAccelX_mss()) + (IMU.getAccelY_mss() * IMU.getAccelY_mss()) + (IMU.getAccelZ_mss() * IMU.getAccelZ_mss()));
  Accel_pitch_deg = asin((float)IMU.getAccelX_mss() / Accel_total_vector_modulus) * Rad_to_deg; // Calculate the pitch angle from accelerometer
  Accel_roll_deg = asin((float)IMU.getAccelY_mss() / Accel_total_vector_modulus) * Rad_to_deg;  // Calculate the roll angle from accelerometer

  /* Accelerometer calibration */

  // To calibrate the accelerometer, put the values ​​0.0 in the variables and uncomment the 3 Serial1.prints
  // When compiling, leave the IMU immobile so that the accelerometer calibrates properly.
  // Once the values ​​are obtained, they are noted and it is recompiled as it had been before.

  Accel_pitch_deg -= -0.85;
  Accel_roll_deg -= 10.55;

  //      Serial1.print(Accel_pitch_deg,6);
  //      Serial1.print("\t");
  //      Serial1.println(Accel_roll_deg,6);

  // If gyroscope and accelerometer are synchronized
  if (Gyro_Accel_sync)
  {
    //  ----- Gyro & accel have been synchronized
    Gyro_pitch_deg = Gyro_pitch_deg * 0.95 + Accel_pitch_deg * 0.05; // Correct the drift of the gyro pitch angle with the accelerometer pitch angle. Original values were 0.9996 and 0.0004m, respectively.
    Gyro_roll_deg = Gyro_roll_deg * 0.95 + Accel_roll_deg * 0.05;    // Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  { //  The 0.98 and 0.02 values are taken from http:// www.pieter-jan.com/node/11else{
    //  ----- Synchronize gyro & accel
    Gyro_pitch_deg = Accel_pitch_deg; // Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll_deg = Accel_roll_deg;   // Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_Accel_sync = true;           // Set the IMU started flag
  }

  // Final corrected Pitch and Roll angle (degrees)
  Pitch_deg = Pitch_deg * 0.9 + Gyro_pitch_deg * 0.1;
  Roll_deg = Roll_deg * 0.9 + Gyro_roll_deg * 0.1;

  // Final corrected Pitch and Roll angle (radians)
  Pitch_rad = -Roll_deg * Deg_to_rad;
  Roll_rad = Pitch_deg * Deg_to_rad;

  // Calculated values of Magnetometer along X and Y axis
  Mag_X_calc = IMU.getMagX_uT() * cos(Pitch_rad) + IMU.getMagY_uT() * sin(Roll_rad) * sin(Pitch_rad) - IMU.getMagZ_uT() * cos(Roll_rad) * sin(Pitch_rad);
  Mag_Y_calc = IMU.getMagY_uT() * cos(Roll_rad) + IMU.getMagZ_uT() * sin(Roll_rad);

  // Correct drif (same as Pitch_deg)
  Mag_X_corrected = Mag_X_corrected * 0.9 + Mag_X_calc * 0.1;
  Mag_Y_corrected = Mag_Y_corrected * 0.9 + Mag_Y_calc * 0.1;

  // The magnetic north has been modified, originally, it was from X to Y (towards which the reading 0 was 90 degrees offset) and without the negative sign of the beginning (towards which the reading would add the angles counterclockwise instead of clockwise).

  Yaw_deg = -atan2(Mag_Y_corrected, Mag_X_corrected) * Rad_to_deg;

  // If you want to be precise, add the declination of the geographical place where you are (https: // www.ign.es/web/gmt-declinacion-magnetica)
  // Yaw_deg += Declination;

  // The next two lines contain the value of Yaw_deg between 0 and 360 degrees.
  if (Yaw_deg < 0)
    Yaw_deg += 360;
  if (Yaw_deg >= 360)
    Yaw_deg -= 360;
}

void show_IMU()
{
  /*
    Function to show IMU_Data on the Serial1 Monitor

    INPUT: 
    None
    OUTPUT: 
    None
  */

  // Print accelerometer values
  //  Serial1.print("Acc: ");
  //  Serial1.print(IMU_accel_data_X, 4);
  //  Serial1.print('\t');
  //  Serial1.print(IMU_accel_data_Y, 4);
  //  Serial1.print('\t');
  // Serial1.print(IMU_accel_data_Z,4);
  // Serial1.println("");

  // Print Gyro values
  // Serial1.print(" / G: ");
  // Serial1.print(IMU_gyro_data_X,4);  Serial1.print('\t');
  // Serial1.print(IMU_gyro_data_Y,4);  Serial1.print('\t');
  // Serial1.print(IMU_gyro_data_Z,4);   Serial1.print('\t');
  // Serial1.print(gyro_Z_offseted,4);
  // Serial1.println("");

  // Print Mag values
  //   Serial1.print("MAGS: ");
  //   Serial1.print(IMU_mag_data_X,4);    Serial1.print(';');
  //   Serial1.print(IMU_mag_data_Y,4);    Serial1.print(';');
  //   Serial1.print(IMU_mag_data_Z,4);
  //   Serial1.println("");

  // Print orientation angles
  Serial1.print(" / D: ");
  Serial1.print(Pitch_deg, 4);
  Serial1.print(';');
  Serial1.print(Roll_deg, 4);
  Serial1.print(';');
  Serial1.print(Yaw_deg, 4);
  Serial1.println("");
}

void read_show_IMU()
{
  /*
    Function to both read and show IMU_Data.

    INPUT: 
    None
    OUTPUT: 
    None, but saves IMU variables and Roll, Pitch and Yaw
  */

  read_IMU();
  show_IMU();
}

// ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ PID Functions
void computePID()
{
  /*
    Function to calculate PID

    INPUT: 
    None
    OUTPUT: 
    None, but saves PID values
  */

  // Time step
  float dT = ((1 / ((float)STM32_CLOCK / (float)Timer1.getPrescaleFactor())) * Timer1.getOverflow()) * 0.001; // in seconds

  read_IMU();

  Current_Yaw_deg = Yaw_deg;

  // Check if PID passes through 0 degrees
  if (Zero_state)
  {
    // Saves if the pid pass through 0, if it does it moves the zone away from 0
    // New variable to not modify the Yaw_deg value
    Current_Yaw_deg = Yaw_deg + 180; // Addition of 180 deg to all values. The error is a substract, so the difference is the same
  }

  if (Current_Yaw_deg >= 360)
    Current_Yaw_deg -= 360; // One of them will increase over 360, it is a correction

  // Percentage
  volatile float Yaw_deg_perc = (Current_Yaw_deg - 360) / (-360) * 100;

  // Transform angle to percentage (Deg_to_reach == setPoint)
  volatile float Deg_to_reach_perc = (Deg_to_reach - 360) / (-360) * 100;

  // Errors
  PID_error = Deg_to_reach_perc - Yaw_deg_perc;       // Calculate error (Proportional)
  PID_cumulative_error += PID_error * dT;             // Calculate the cumulative error (Integral)
  PID_rate_error = (PID_error - PID_last_error) / dT; // Calculate the rate of error (Derivative)

  // PID Control
  float PID_P = kp * PID_error;            // Proportional
  float PID_I = ki * PID_cumulative_error; // Integral
  float PID_D = kd * PID_rate_error;       // Derivative

  /*
    // Limit 
    // Ensure not overflow
    if (PID_P > 255)
      PID_P = 255;
    if (PID_P < -255)
      PID_P = -255;
    if (PID_I > 255)
      PID_I = 255;
    if (PID_I < -255)
      PID_I = -255;
    if (PID_D > 255)
      PID_D = 255;
    if (PID_D < -255)
      PID_D = -255;
  */

  PID_output = PID_P + PID_I + PID_D; //  PID control

  // Prevent overflow
  if (PID_output > 255)
    PID_output = 255;
  if (PID_output < -255)
    PID_output = -255;

  bool RW_direction = true; // true=positive (CounterClockwise), false=negative (Clockwise)

  if (PID_output < 0)
  {
    PID_output = -PID_output;
    RW_direction = false;
  }

  set_impulse(RW_direction, PID_output);

  // Save current error and time for next iteration
  PID_last_error = PID_error; // Save current error
}

// ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ MODES OF OPERATION FUNCTIONS
void mode_Select(int mode_value)
{
  /*
    Function to select Mode

    INPUT: 
    mode_value ['0': Waiting; '1': Positioning; '2': Reading]
    OUTPUT: 
    None, but exits to selected mode
  */

  switch (mode_value)
  {
  default: // Mode OBC Input Waiting (0): Waiting for OBC
    Serial1.println("Reading mode from OBC");
    mode_OBC_Input_Wait();
    break;
  case 1: // Mode Positioning RW only
    Serial1.println("Mode Positioning");
    mode_Positioning_RW();
    break;
  case 2: // Mode IMU reading (TEST mode)
    Serial1.println("Reading IMU");
    mode_IMU_reading();
    break;
  case 3: // Mode Motor ON OFF (TEST mode)
    Serial1.println("Mode motor ON OFF");
    mode_motor_on_off();
    break;
  }
}

void mode_motor_on_off()
{
  /*
    TEST PURPOSE Function

    INPUT: 
    None
    OUTPUT: 
    None, but saves IMU variables and Roll, Pitch and Yaw
  */

  OBC_mode_value = 0;
  set_impulse(0, 255);
  Timer1.attachInterrupt(TIMER_CH3, OBC_data_receive);
  while (OBC_data_value == 0)
  {
    delay(1); // If not used the while function does not work
  }
  Timer1.detachInterrupt(TIMER_CH3);
  set_impulse(0, 0);
  OBC_data_value = 0;
  mode_OBC_Input_Wait();
}

void mode_IMU_reading()
{
  /*
    TEST PURPOSE Function

    INPUT: 
    None
    OUTPUT: 
    None, but saves IMU variables and Roll, Pitch and Yaw
  */

  OBC_mode_value = 0;
  read_show_IMU();
  mode_Select(OBC_mode_value);
}

void mode_OBC_Input_Wait()
{
  /*
    0. Default mode, OBC Reading

    INPUT: 
    None
    OUTPUT: 
    None, but exits to mode_Select
  */
  if (Stop_state != 0)
  {
    Stop_state = 0;
    Timer1.detachInterrupt(TIMER_CH3);
  }
  OBC_mode_value = 0;
  Timer1.attachInterrupt(TIMER_CH3, OBC_mode_receive);
  while (OBC_mode_value == 0)
  {
    delay(1); // If not used the while function does not work
    // it can be added more conditions to evade being blocked until a data is received.
    // for example, it could function an interrupt with a forced exit and an if after or something
  }
  Timer1.detachInterrupt(TIMER_CH3);
  Serial1.println(OBC_mode_value);
  mode_Select(OBC_mode_value);
}

void mode_Positioning_RW()
{
  /*
    1. Mode Positioning (RW only)

    INPUT: 
    None
    OUTPUT:
    None, but exits to suitable positioning type
  */

  OBC_mode_value = 0;
  Serial1.println("Positioning begin");
  // Insert code to select if the positioning will be coarse (impulses) or fine (PD)
  /* It should be something like:
 * read IMU degree value
 * get OBC value from OBC and save in global variable
 * comparison to OBC value (wanted position), add to the void function definition
 * if (comparison<acceptable_error){
 * positioning_fine();
 * }else{
 * positioning_coarse();
 * }
 */
  read_IMU();
  Serial1.print("Axis Z position: ");
  Serial1.println(Yaw_deg);
  Serial1.println("Insert degree value to turn");
  OBC_data_value = 0;
  Timer1.attachInterrupt(TIMER_CH3, OBC_data_receive);
  while (OBC_data_value == 0)
  {
    delay(1); // If not used the while function does not work
  }
  Timer1.detachInterrupt(TIMER_CH3);

  Serial1.print("Value to turn: "); // In this case turn, but can be what is needed
  Serial1.print(OBC_data_value);

  // Initiate EmergencyStop routine
  Timer1.attachInterrupt(TIMER_CH3, EmergencyStop);

  // Depend on the tolerance enable Fine or Coarse positioning
  if (abs(OBC_data_value) <= Pointing_mode_tolerance)
  {
    positioning_Fine();
  }
  else
  {
    positioning_Coarse();
  }
}

void positioning_Coarse()
{
  /*
    1.1. Mode Positioning Coarse: For now simple

    INPUT: 
    None
    OUTPUT:
    None, but exits to positioning_Fine or mode_select
  */

  Serial1.println("Mode Positioning Coarse");

  // Stores a new variable to not overwrite the original value, also for motor as it needs a positive value and direction
  float degree_turn_value = OBC_data_value;
  bool RW_direction = true; // true=positive (CCW), false=negative (CW) along CubeSat's z-axis
                            // In case OBC_data_value is positive, value remains the same and rw direction is true, the default value of the variable
                            // In case OBC_data_value is negative, value change to positive and rw direction is false

  if (degree_turn_value < 0)
  {
    degree_turn_value = -degree_turn_value;
    RW_direction = false;
  }

  float Initial_IMU_degree_value, Final_IMU_degree_value, Delta_degree_ramp, Degree_stop_wait;
  float Prev_Yaw_deg, Check_Yaw_deg;
  int overlap_count = 0;
  bool waiting = true;

  // ----------------------------------------------------------ACCELERATION

  read_show_IMU();
  Initial_IMU_degree_value = Yaw_deg; // Stores initial degree, only in Z
  int initial_RW_speed = RW_speed;
  generate_ramp(RW_direction, 0, 20, 1);
  // 0 will not be used as we dont define the time the motor lasts to do an impulse. 255 is the max value
  // This values will be substituted by how we want the time and speed to reach in the ramp.
  read_IMU();
  Final_IMU_degree_value = Yaw_deg; // Stores final degree, only in Z

  Delta_degree_ramp = Final_IMU_degree_value - Initial_IMU_degree_value; // degree turnt on acc, only in Z.

  if (RW_direction == false) // RW Clockwise
  {
    Delta_degree_ramp = Final_IMU_degree_value - Initial_IMU_degree_value; // degree turnt on acc, only in Z. Should be positive
  }

  if (RW_direction)
  {
    Delta_degree_ramp = Final_IMU_degree_value - Initial_IMU_degree_value; // degree turnt on acc, only in Z. Should be positive
  }
  else
  {
    Delta_degree_ramp = -(Final_IMU_degree_value - Initial_IMU_degree_value); // degree turnt on acc, only in Z. Should be positive
  }

  if (Delta_degree_ramp < 0)
  {
    Delta_degree_ramp += 360; // In case its negative adds 360
    // Negative cases: changes goes by 0º. EX: 330 to 30 when CCW (should be 60 but calculus is 30-330= -300)
    //                                     EX: 30 to 330 when CW  (should be 60 but calculus is 30-330= -300)
  }
  Serial1.print("ID: ");
  Serial1.print(Initial_IMU_degree_value);
  Serial1.print(" / T: ");
  Serial1.print(Delta_degree_ramp); // Difference of ange
  Serial1.print(" / ED: ");
  Serial1.println(Final_IMU_degree_value); // Final, End angle
  // --------------------------------------------------------------WAITING
  if ((degree_turn_value - 2 * Delta_degree_ramp) > 0) // if it is <0, skip the waiting phase, deceleration must be done immediately after, and still it would be too much turn.
  {
    waiting = true;
    if (RW_direction)
    {
      // CASE CCW

      /*
      RW CCW
      CB CW
      IMU positive

      Angulo inicial 300º

      gira 150º en sentido CB CW = degree_turn_value

      angulo final acc 350º = Final_IMU_degree_value

      Delta_degree_ramp=350-300=50

      Degree_stop_wait=350+(150-2*50)=400;  40

      */
      Degree_stop_wait = Final_IMU_degree_value + (degree_turn_value - 2 * Delta_degree_ramp); // Get value of degree to start

      Timer1.attachInterrupt(TIMER_CH4, read_IMU);
      Serial1.println("Waiting");
      while (waiting)
      { // Stays as long as waiting is true.

        // CAUTION WITH READING VALUES; AS IT IS ALWAYS FROM 0 TO 360
        //  Degree_stop_wait will always be > Yaw_deg. If Yaw_deg>> (ex: 359º),  Degree_stop_wait can be >360. Thus the value of If Yaw_deg would never reach Degree_stop_wait
        //  Degree_stop_wait cant be decreased, as the way to check if its reached is by a greater. If it is decreased by 360º (so it stays in relative place), it could be < Yaw_deg and would immediately exit the while without waiting.
        //  the way to do is check if there is a heavy change on Yaw_deg (pass on 0) to check if adding or substracting a lap, making it go out of the 0 and 360 range.
        // It should not be a high acceleration enough to make a jump of degree of 180º in such short time,so it should be fine.
        // Caution disconnection from IMU, could give a false jump, that is why a check on Yaw_deg first.

        if (Yaw_deg < 0.0001 && Yaw_deg > -0.0001)
        { // if it is almost exactly 0 then most probably there is a disconnection. Close numbers should not trigger it.
        }
        else if (Yaw_deg - Prev_Yaw_deg < -180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 359 to 0.
          overlap_count += 1; // adds a lap
        }
        else if (Yaw_deg - Prev_Yaw_deg > +180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 0 to 359. (Not possible in theory, as it increases, but deviations could mess it up)
          overlap_count -= 1; // substracts a lap
        }                     // if neither are triggered, change has been samall or no change has been done yet
        Prev_Yaw_deg = Yaw_deg;
        Check_Yaw_deg = Yaw_deg + (360 * overlap_count); // Rewriting of value, in new variable so it does not add itself.
        if (Check_Yaw_deg > Degree_stop_wait)
        {                  // As it is CCW, degree increases. When reading is > to stop value, exits the while
          waiting = false; // exit condition
        }
        delay(1); // To solve errors
        // No writing of read_IMU as it is already done by a timer. Just to remind Yaw_deg is constantly reading values.
      }
      Timer1.detachInterrupt(TIMER_CH4);
    }
    else
    {
      // CASE CW
      // if its <0, it must be done immediately after, and still it would be too much turn.
      Degree_stop_wait = Final_IMU_degree_value - (degree_turn_value - 2 * Delta_degree_ramp); // Get value of degree to start

      Timer1.attachInterrupt(TIMER_CH4, read_IMU);
      while (waiting)
      { // Stays as long as waiting is true.
        Serial1.println("Waiting");

        // CAUTION WITH READING VALUES; AS IT IS ALWAYS FROM 0 TO 360
        //  Degree_stop_wait will always be < Yaw_deg. If Yaw_deg<< (ex: 1º),  Degree_stop_wait can be <0. Thus the value of If Yaw_deg would never reach Degree_stop_wait
        //  Degree_stop_wait can't be increased, as the way to check if its reached is by a greater. If it is increased by 360º (so it stays in relative place), it could be > Yaw_deg and would immediately exit the while without waiting.
        //  the way to do is check if there is a heavy change on Yaw_deg (pass on 0) to check iff adding or substracting a lap, making it go out of the 0 and 360 range.
        // It should not be a high acceleration enough to make a jump of degree of 180º in such short time, so it should be fine.
        // Caution disconnection from IMU, could give a false jump, that is why a check on Yaw_deg first.

        Check_Yaw_deg = Yaw_deg; // New variable so it does not change during an iteration
        if (Check_Yaw_deg < 0.0001 && Check_Yaw_deg > -0.0001)
        { // if it is almost exactly 0 then most probably there is a disconnection. Close numbers should not trigger it.
        }
        else if (Check_Yaw_deg - Prev_Yaw_deg > +180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 359 to 0. (Not possible in theory, as it decreases, but deviations could mess it up)
          overlap_count += 1; // adds a lap
        }
        else if (Check_Yaw_deg - Prev_Yaw_deg < -180)
        {                     // Checks for a heavy change (greater than half turn). Done by changes in 0, like jumping from 0 to 359.
          overlap_count -= 1; // substracts a lap
        }                     // if neither are triggered, change has been samall or no change has been done yet

        Prev_Yaw_deg = Check_Yaw_deg;
        Check_Yaw_deg = Check_Yaw_deg + (360 * overlap_count); // Rewriting of value. It does not add itself as it gets the read value in each iteration. In other words, it will not go through the iteration with numbers outside 0 and 360 range
        if (Check_Yaw_deg < Degree_stop_wait)
        {                  // As it is CCW, degree increases. When reading is > to stop value, exits the while
          waiting = false; // exit condition
        }
        delay(1); // To solve errors
        // No writing of read_IMU as it is already done by a timer. Just to remind Yaw_deg is constantly reading values.
      }
      Timer1.detachInterrupt(TIMER_CH4);
    }
  }

  // ----------------------------------------------------------DECELERATION
  Serial1.print("ID: ");
  Serial1.print(Final_IMU_degree_value);
  Serial1.print(" / ED: ");
  Serial1.println(Degree_stop_wait);

  // Initial_RW_Speed returns speed to original value instead of 0
  generate_ramp(RW_direction, initial_RW_speed, 0, 0);
  // This is to assure the impulse is the same, as if there is some momentum accumulation, returning to 0 would be a different impulse
  // 0 will not be used as we dont define the time the motor lasts to do an impulse. 255 is the max value
  // This values will be substituted by how we want the time and speed to reach in the ramp.

  read_IMU();
  Final_IMU_degree_value = Yaw_deg; // can be overwritten, final degree of manoeuvre
  if (RW_direction)
  {
    Delta_degree_ramp = Final_IMU_degree_value - Initial_IMU_degree_value; // Should be positive
  }
  else
  {
    Delta_degree_ramp = Initial_IMU_degree_value - Final_IMU_degree_value; // Should be positive
  }
  if (Delta_degree_ramp < 0)
  {                           // can be overwritten, real turn of manoeuvre
    Delta_degree_ramp += 360; // In case its negative adds 360
    // Negative cases: changes goes by 0º. EX: 330 to 30 when CCW (should be 60 but calculus is 30-330= -300)
    //                                     EX: 30 to 330 when CW  (should be 60 but calculus is 30-330= -300)
  }

  Serial1.print("ID: ");
  Serial1.print(Degree_stop_wait);
  Serial1.print(" / T: ");
  Serial1.print(Delta_degree_ramp);
  Serial1.print(" / ED: ");
  Serial1.println(Final_IMU_degree_value);

  if ((Delta_degree_ramp - degree_turn_value) < Final_pointing_tolerance && (Delta_degree_ramp - degree_turn_value) > Final_pointing_tolerance)
  {
    // Detach EmergencyStop
    Timer1.detachInterrupt(TIMER_CH3);

    // Real turn vs wanted turn, checks if it is considered good
    mode_OBC_Input_Wait(); // Valid position
  }
  else
  {
    positioning_Fine(); // Correction
  }
}

void positioning_Fine()
{
  /*
    1.2. Mode Positioning Fine: PID

    INPUT: 
    None
    OUTPUT:
    None, but exits to mode_select
  */

  Serial1.println("Mode Positioning Fine");

  bool waiting = true;

  Deg_to_reach = OBC_data_value + Yaw_deg; // Get value to reach, contained in 360
  if (Deg_to_reach < 0)
  {
    Deg_to_reach += 360;
    Zero_state = true; // Used to store if it passes 0.
    // A PD passing through 0 could give great problems, as it has a very big change in value. Further used in computePID()
  }
  else if (Deg_to_reach >= 360)
  {
    Deg_to_reach -= 360; // These two lines contain the value of the Yaw_deg in 0-360 degrees.
    Zero_state = true;
  }
  else
  {
    Zero_state = false;
  }

  if (Zero_state)
  {
    Deg_to_reach + 180;
    if (Deg_to_reach >= 360)
      Deg_to_reach -= 360;
  }

  Timer1.attachInterrupt(TIMER_CH4, computePID);
  while (waiting)
  { // Range of tolerance
    if (abs(IMU_gyro_data_Z) < Gyro_tolerance && abs(IMU_accel_data_X) < Accel_tolerance)
    { // we consider it is stopped, modify values to be accurate
      waiting = false;
    }
    delay(1); // Change
  }
  waiting = true;
  Timer1.detachInterrupt(TIMER_CH4);

  // At exit, RW_speed could not be 0
  OBC_data_value = 0;
  Serial1.println("End of manoeuvre");

  // Detach EmergencyStop
  Timer1.detachInterrupt(TIMER_CH3);

  mode_Select(OBC_mode_value);
}

// ██████████████████████████████████████████████████████████████████████ VOID SETUP
void setup()
{
  delay(1000); //  wait to let open the serial
  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Timers
  Timer1.pause();
  Timer1.setPrescaleFactor(7200); // 72MHz Clock / 7200 = 10KHz timer
  Timer1.setOverflow(1000);       // Overflow occurs at 1000, each 100 ms timer restarts

  Timer1.setMode(TIMER_CH3, TIMER_OUTPUT_COMPARE); // Configure channel to OUTPUTCOMPARE: Channel for OBC read values
  Timer1.setMode(TIMER_CH4, TIMER_OUTPUT_COMPARE); // Channel for IMU read values
  Timer1.setCompare(TIMER_CH3, 1);                 // Phase value in Overflow range
  Timer1.setCompare(TIMER_CH4, 1);

  Timer1.refresh(); // Refresh timer and start over
  Timer1.resume();

  // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Initiations
  Serial1.begin(9600);
  SPI.begin();

  Serial1.println("START");

  // ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓ Setups

  pinMode(LEDPIN, OUTPUT); // Integrated LED
  pinMode(CS1, OUTPUT);    // NCS Pin definition

  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ Driver Setup
  Serial1.println("Configuring Driver...");
  DriverOne.settings.commInterface = I2C_MODE; // Driver Comm Mode
  DriverOne.settings.I2CAddress = 0x5D;        // Driver Address (0x5D by Default)

  while (DriverOne.begin() != 0xA9)
  { // Driver wait for idle
    Serial1.println("ID Mismatch");
    delay(200);
  }
  Serial1.println("ID Match");

  Serial1.println("Waiting for enumeration"); // Driver wait for peripherals
  while (DriverOne.ready() == false)
    ;
  Serial1.println("Ready");

  while (DriverOne.busy())
    ; // Driver enable
  DriverOne.enable();
  Serial1.println("Driver Ready to Use!");

  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Setup
  Serial1.println("Configuring MPU9250...");
  int status = IMU.begin();
  if (status < 0)
  {
    Serial1.println("IMU initialization unsuccessful");
    Serial1.println("Check IMU wiring or try cycling power");
    Serial1.print("Status: ");
    Serial1.println(status);
    while (1)
    {
    }
  }

  IMU.setGyroRange(IMU.GYRO_RANGE_500DPS);

  // ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒ IMU Calibration
  // Use for getting the values ​​on calibration setting below

  // To calibrate the magnetometer or the compass, everything is commented except the last 3 lines.
  // It moves in the shape of an eight approximately 2 min. It is recommended to carry out several times until the measurements are fine-tuned.
  // Uncomment everything and enter the values ​​obtained from the MagBias and ScaleFactor to view the results of the magnetometer.
  //
  //    IMU.calibrateMag();
  //    Serial1.println("Done");
  //
  //    Serial1.print(IMU.getMagBiasX_uT());
  //    Serial1.print(",");
  //    Serial1.print(IMU.getMagBiasY_uT());
  //    Serial1.print(",");
  //    Serial1.println(IMU.getMagBiasZ_uT());
  //
  //    Serial1.print(IMU.getMagScaleFactorX());
  //    Serial1.print(",");
  //    Serial1.print(IMU.getMagScaleFactorY());
  //    Serial1.print(",");
  //    Serial1.println(IMU.getMagScaleFactorZ());

  IMU.setMagCalX(24.48, 1.26); // The first value corresponds to the MagBias, and the second the ScaleFactor.
  IMU.setMagCalY(11.26, 0.86);
  IMU.setMagCalZ(-24.11, 0.96);

  Serial1.println("MPU9250 Ready to Use!");

  Serial1.println("Reading mode from OBC");
  mode_OBC_Input_Wait();
}

// ██████████████████████████████████████████████████████████████████████ VOID LOOP
void loop()
{
}
