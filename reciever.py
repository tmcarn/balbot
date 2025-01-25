# from flySkyiBus import IBus


# bus = IBus('/dev/ttyAMA10')  # use your serial port name here


# data = bus.read()  # Read data from serial port
# print(data)  # print the data read from the serial port


import serial

ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

while True:
    data = ser.readline(32)
    print(data)