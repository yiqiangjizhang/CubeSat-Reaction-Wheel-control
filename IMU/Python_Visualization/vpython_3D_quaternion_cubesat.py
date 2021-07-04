# The following code rotates a reference frame

# Libraries
from vpython import *  # vPython library
import numpy as np  # Numpy library
import time  # Time library
import math  # Math library

# Libraries
import serial

# Code
arduinoData = serial.Serial('com3', 115200)  # COM and baud rate

time.sleep(1)  # Delay of 1 s to ensure the serial port working

# View orientation
scene.forward = vector(-1, -1, -1)

scene.width = 600
scene.height = 600


# Adding the Arduino
breadBoard = box(length=6, width=6, height=6,
                 color=color.white, opacity=0.25)

# Compound object
object = compound([breadBoard])

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

    # Quaternions
    q0 = float(data[0])
    q1 = float(data[1])
    q2 = float(data[2])
    q3 = float(data[3])

    # Convert quaternions to euler angles
    roll = -atan2(2*(q0*q1+q2*q3), 1-2*(q1**2+q2**2))
    pitch = asin(2*q0*q2-q3*q1)
    yaw = -atan2(2*(q0*q3+q1*q2), 1-2*(q2**2+q3**2))

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
