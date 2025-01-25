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

        roll = math.radians(attitude['angx'])
        pitch = math.radians(attitude['angy'])
        yaw = math.radians(attitude['heading'])

        return pitch # in radians
    

        
    

# # FOR TESTING
# imu = IMU()

# while True:
#     print(imu.get_pitch())