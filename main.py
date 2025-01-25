from imu import IMU
from motor_controller import MotorController
from pid import PID
from reciever import Reciever

import math
import time
import numpy as np

MAX_PITCH = math.pi / 8 # ~ 22 degrees
DEADBAND = 0.05 # ~ 3 degrees

imu = IMU()

motor1 = MotorController(26)
motor2 = MotorController(12)

pid = PID()
reciever = Reciever()

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
        motor1.kill_motor() # stops motor
        break 

    # elif np.abs(pitch - pid.setpoint) < DEADBAND:
    #     motor1.set_motor_speed(0.05) # Temporarily stops motor -> balancing correctly
    #     continue

    motor_value = pid.compute(pitch, dt)
    motor1.set_motor_speed(motor_value)



    