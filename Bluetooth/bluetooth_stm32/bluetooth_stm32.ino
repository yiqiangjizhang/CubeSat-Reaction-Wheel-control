/*
 * UART Bluetooth Communication with STM32
 */

// Libraries
#include <Arduino.h>
#include <time.h>

// Parameters
char inputData_serial1 = 0; // Initialize input data from Serial 1
char inputData_serial = 0;  // Initialize input data from Serial

// Main setup
void setup()
{
  Serial1.begin(9600);                      // Begin UART serial port
  Serial.begin(9600);                       // Begin Serial port
  Serial1.println("Bluetooth initialized"); // Print initilize message
}

// Main loop
void loop()
{
  // Check if serial is available
  if (Serial1.available() > 0)
  {
    inputData_serial1 = Serial1.read();
    Serial.write(inputData_serial1); // Display message throuhg Serial port
  }
  // If Serial
  if (Serial.available() > 0)
  {
    inputData_serial = Serial.read();
    Serial1.write(inputData_serial); // Display message throuhg Serial 1 port
  }

  // Delay 100 ms
  delay(100);
}
