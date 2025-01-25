from flySkyiBus import IBus

bus = IBus('/dev/ttyAMA10')  # use your serial port name here

while True:
    data = bus.read()  # Read data from serial port
    print(data)  # print the data read from the serial port

    print("running")

    