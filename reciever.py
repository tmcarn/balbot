from flySkyiBus import IBus
import numpy as np


bus = IBus('/dev/ttyAMA0')  # use your serial port name here

while True:
    data = np.array(bus.read())  # Read data from serial port
    print(data[2:8])  # print the data read from the serial port


# import serial

# ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

# while True:
#     data = ser.readline(32)
#     print(data)