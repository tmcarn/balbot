from imu import IMU
from motor_controller import MotorController
from pid import PID
from reciever import Reciever

import math
import time
import numpy as np

MAX_PITCH = math.pi / 2 # ~ 90 degrees
DEADBAND = 0.05 # ~ 3 degrees

imu = IMU()

# motor1 = MotorController(26)
# motor2 = MotorController(12)

pid = PID()
reciever = Reciever()

pid.set_constants(0.001,0,0)

prev_time = time.time()
current_time = 0
dt = 0


while True:
    pitch = imu.get_pitch() # in radians

    if pitch == None:
        print("No IMU Reading")
        continue
    
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    motor_value = pid.compute(pitch, dt)
    # motor1.set_motor_speed(motor_value)
    # motor2.set_motor_speed(motor_value)
    time.sleep(0.01)



    