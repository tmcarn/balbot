from imu import IMU
from motor_controller import MotorController

import math

imu = IMU()
for i in range(1000):
    print(math.degrees(imu.get_pitch()))
