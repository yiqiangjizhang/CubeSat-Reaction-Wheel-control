# This code imports ARDUINO data into python

# Libraries
import time
import serial

# Code
arduinoData = serial.Serial('com5', 115200)  # COM and baud rate

time.sleep(1)  # Delay of 1 s to ensure the serial port working

# Loop forever
while True:
    # Loop until there is data into serial port (hang on until there is data)
    while (arduinoData.inWaiting() == 0):
        pass

    # Since there is data in serial port, read the data
    data = arduinoData.readline()
    data = str(data, 'utf-8')  # Format the data
    data = data.split(',')  # Split the data into a list

    # print(data)

    # Extract values from the accelerometer
    acc_x = float(data[0])
    acc_y = float(data[1])
    acc_z = float(data[2])

    # Extract filtered values (Complementary filter) of pitch and roll
    pitch = float(data[11])
    roll = float(data[12])

    # Extract yaw
    yaw = float(data[13])

    print(
        f"acc_x= {acc_x}, acc_y= {acc_y}, acc_z= {acc_z}, pitch= {pitch}, roll= {roll}, yaw= {yaw}")
