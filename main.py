from imu import IMU
from motor_controller import MotorController
from pid import PID

import math
import time
import numpy as np

MAX_PITCH = math.pi / 8 # ~ 22 degrees

imu = IMU()
motor = MotorController(26)
pid = PID()

pid.set_constants(5,0,0)


prev_time = time.time()
current_time = 0
dt = 0

while True:
    pitch = imu.get_pitch() # in radians
    
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    if np.abs(pitch) > MAX_PITCH:
        motor.kill_motor() # stops motor
        break 

    motor_value = pid.compute(pitch, dt)
    motor.set_motor_speed(motor_value)



    