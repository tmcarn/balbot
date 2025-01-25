from pymultiwii import MultiWii
import time
import numpy as np
import math

PITCH_OFFSET = -0.0837758040957278


class IMU():
    def __init__(self) -> None:
        self.board = MultiWii('/dev/ttyUSB0')
        print("IMU initialized")

    def get_pitch(self):
        # Reading in Orientation Data
        attitude = self.board.getData(MultiWii.ATTITUDE)

        # pitch = math.radians(attitude['angx'])
        roll = math.radians(attitude['angy']) - PITCH_OFFSET
        # yaw = math.radians(attitude['heading'])

        return roll # in radians