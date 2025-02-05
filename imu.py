from pymultiwii import MultiWii
import time
import numpy as np
import math

PITCH_OFFSET = 0


class IMU():
    def __init__(self) -> None:
        self.board = MultiWii('/dev/ttyUSB0')
        print("IMU initialized")

    def get_pitch(self):
        # Reading in Orientation Data
        attitude = self.board.getData(MultiWii.ATTITUDE)

        if attitude == None:
            return None
        
        roll = attitude['angx']
        pitch = attitude['angy']
        yaw = attitude['heading']

        return roll # in radians
    

