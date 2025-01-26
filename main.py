from imu import IMU
from motor_controller import MotorController
from pid import PID
from steering_controller import Steering_Controller

import math
import time
import numpy as np

PIN_1 = 26
PIN_2 = 12

LOOP_DELAY = 0.01

imu = IMU()

motor1 = MotorController(PIN_1)
motor2 = MotorController(PIN_2)

pid = PID()
reciever = Steering_Controller()

pid.set_constants(0.5,0,0)

prev_time = time.time()
current_time = 0
dt = 0

running = True

while True:
    pitch = imu.get_pitch() # in radians

    if pitch == None:
        print("No IMU Reading")
        continue

    elif np.abs(pitch)> 45:
        break
    
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    motor_value = pid.compute(pitch, dt)
    motor1.set_motor_speed(motor_value)
    motor2.set_motor_speed(motor_value)
    time.sleep(LOOP_DELAY)


motor1.kill_motor()
print("Loop Terminated")


    