# The following code rotates a reference frame

# Libraries
from vpython import *  # vPython library
import numpy as np  # Numpy library
import time  # Time library
import math  # Math library

# Libraries
import serial

# Code
arduinoData = serial.Serial('com5', 115200)  # COM and baud rate

time.sleep(1)  # Delay of 1 s to ensure the serial port working

# Conversions
deg2rad = 2*np.pi/360
rad2deg = 1/deg2rad

# View orientation
scene.forward = vector(-1, -1, -1)

scene.width = 600
scene.height = 600


# Adding the Arduino
breadBoard = box(length=6, width=2, height=0.2,
                 color=color.white, opacity=0.25)

arduino = box(length=1, width=0.5, height=0.1,
              color=color.green, pos=vector(-2, 0.1 + 0.05, 0))

IMU = box(length=1, width=0.75, height=0.1,
          color=color.blue, pos=vector(1.5, 0.1 + 0.05, 0))

# Compound object
object = compound([breadBoard, arduino, IMU])

x_arrow = arrow(length=4, shaftwidth=0.25,
                color=color.red, axis=vector(1, 0, 0))
y_arrow = arrow(length=4, shaftwidth=0.25,
                color=color.green, axis=vector(0, 1, 0))
z_arrow = arrow(length=4, shaftwidth=0.25,
                color=color.cyan, axis=vector(0, 0, 1))

x_body_arrow = arrow(length=6, shaftwidth=0.25,
                     color=color.purple, axis=vector(1, 0, 0))
y_body_arrow = arrow(length=6, shaftwidth=0.25,
                     color=color.magenta, axis=vector(0, 1, 0))
z_body_arrow = arrow(length=6, shaftwidth=0.25,
                     color=color.white, axis=vector(0, 0, 1))

# Loop forever
while True:

    # Loop until there is data into serial port (hang on until there is data)
    while (arduinoData.inWaiting() == 0):
        pass

    # Since there is data in serial port, read the data
    data = arduinoData.readline()
    data = str(data, 'utf-8')  # Format the data
    data = data.split(',')  # Split the data into a list

    # Extract filtered values (Complementary filter) of pitch and roll
    pitch = float(data[11])*deg2rad
    roll = float(data[12])*deg2rad

    # Extract yaw
    yaw = float(data[13])*deg2rad + np.pi

    # for yaw in np.arange(0, 2*np.pi, 0.01):
    rate(50)  # 50fps
    k = vector(cos(yaw)*cos(pitch),
               sin(pitch), sin(yaw)*cos(pitch))

    y = vector(0, 1, 0)
    s = cross(k, y)  # Side vector
    v = cross(s, k)  # Up vector

    vrotate = v*cos(roll) + cross(k, v)*sin(roll)

    x_body_arrow.axis = k
    z_body_arrow.axis = cross(k, vrotate)
    y_body_arrow.axis = vrotate
    object.axis = k
    object.up = vrotate  # Up direction
    x_body_arrow.length = 4
    z_body_arrow.length = 4
    y_body_arrow.length = 4
