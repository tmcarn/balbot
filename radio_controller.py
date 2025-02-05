from flySkyiBus import IBus
import numpy as np

class RadioController():
    def __init__(self) -> None:
        self.bus = IBus('/dev/ttyAMA0')

    def raw_inputs(self):
        data = np.array(self.bus.read())  # Read data from serial port
        print(f"DATA: {data}")
        return(data[2:8])  # Returns only data from our 6 channels
    
    def scaled_input(self):
        scaled_inputs = (self.raw_inputs() - 1500) / 500 # -1 to 1
        return scaled_inputs
    
    def pid_constants(self, max_val):
        scaled_inputs = (self.raw_inputs() - 1000) / 1000 # 0 to 1

        # Scaled from 0 to max_val
        kp = scaled_inputs[2] * max_val
        ki = scaled_inputs[4] * max_val
        kd = scaled_inputs[5] * max_val

        print(f"PID VALUES: {(kp, ki, kd)}")

        return (kp, ki, kd)

