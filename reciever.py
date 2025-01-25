from flySkyiBus import IBus
import numpy as np


class Reciever():
    def __init__(self) -> None:
        self.bus = IBus('/dev/ttyAMA0')  # use your serial port name here

    def get_inputs(self):
        data = np.array(bus.read())  # Read data from serial port

        print(data)
        return(data[2:8])  # print the data read from channels 1-6


rec = Reciever()

while True:
    rec.get_inputs()