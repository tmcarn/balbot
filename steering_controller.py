from flySkyiBus import IBus
import numpy as np

MAX_LR = 1
MAX_FB = 1

CONTROLLER_MIN = 1000
CONTROLLER_MAX = 2000




class Steering_Controller():
    def __init__(self) -> None:
        self.bus = IBus('/dev/ttyAMA0')  # use your serial port name here

    def get_inputs(self):
        data = np.array(self.bus.read())  # Read data from serial port
        return(data[2:8])  # print the data read from channels 1-6
    
    # def get_steering_input(self):
    #     # y 2000 -> 1 1000 -> -1
    #     inputs = self.get_inputs()
    #     LR = 

