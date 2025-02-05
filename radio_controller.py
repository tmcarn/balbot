from flySkyiBus import IBus
import numpy as np
import time

class RadioController():
    def __init__(self) -> None:
        self.bus = IBus('/dev/ttyAMA0')

    def raw_inputs(self):
        self.bus.serial.reset_input_buffer()  # Flush out old data
        data = np.array(self.bus.read())  # Read data from serial port
        return(data[2:8])  # Returns only data from our 6 channels
    
    # def scaled_input(self):
    #     scaled_inputs = (self.raw_inputs() - 1500) / 500 # -1 to 1
    #     return scaled_inputs
    
    def scaled_input(self, channel, max_val):
        scaled_inputs = (self.raw_inputs() - 1000) / 1000 # 0 to 1

        # Scaled from 0 to max_val
        return scaled_inputs[channel] * max_val
