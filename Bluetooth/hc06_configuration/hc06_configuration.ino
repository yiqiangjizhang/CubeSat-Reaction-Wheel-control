/* HC06 bluetooth module setup
*/

// Libraries
#include <SoftwareSerial.h>   // This library enables bluetooth communcication

// HC06 parameters
const int Rx = 2;            // Connect Arduino Digital Pin 2 to HC06 Pin Tx
const int Tx = 3;            // Connect Arduino Digital Pin 3 to HC06 Pin Rx
SoftwareSerial hc06(Rx, Tx); // Set Receive and Transmission pins

// Configuration parameters
char BPS = '4';                    // 1=1200 , 2=2400, 3=4800, 4=9600, 5=19200, 6=38400, 7=57600, 8=115200
char NAME_ID[20] = "HC06_Plathon"; // 30 characters maximum
char PASS[10] = "1234";            // 4 bit number

// Setup
void setup()
{
  //Initialize Serial Monitor
  Serial.begin(9600);

  //Initialize Bluetooth Serial Port
  hc06.begin(9600);
  
  // AT commands for configuration
  hc06.print("AT"); // Initiate sequence 
  delay(1000); // Wait 1 second
   
  hc06.print("AT+NAME"); 
  hc06.print(NAME_ID); // Configure bluetooth device name
  delay(1000); // Wait 1 second
   
  hc06.print("AT+BAUD"); 
  hc06.print(BPS); // Transmission baud rate velocity
  delay(1000); // Wait 1 second
   
  hc06.print("AT+PIN");
  hc06.print(PASS); // Set Password
  delay(1000); // Wait 1 second

  // Print setup message
  Serial.println("Module set up");
  Serial.print("PIN:");
  Serial.println(PASS);
  Serial.print("Name: ");
  Serial.println(NAME_ID);

  // Wait 1000 ms to ensure the sensor starts correctly
  delay(1000);
}

// Send and receive data
void loop()
{
  //Write data from HC06 to Serial Monitor
  if (hc06.available())
  {
    Serial.write(hc06.read());
  }

  //Write from Serial Monitor to HC06
  if (Serial.available())
  {
    hc06.write(Serial.read());
  }

}
