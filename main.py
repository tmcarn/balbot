from imu import IMU
from motor_controller import MotorController
from pid import PID
from radio_controller import RadioController

import math
import time
import numpy as np

LOOP_DELAY = 0.01 # ~100 loops per second

KILL_ANGLE = 60 # degrees

# Initialization
imu = IMU()
motors = MotorController()
pid = PID()
pid.setpoint = -0.5
# reciever = RadioController()

current_time = None
prev_time = time.time()
dt = None

running = True

while running:
    pitch = imu.get_pitch() # in degrees

    print(f'Angle: {pitch} degrees')

    if pitch == None:
        print("No IMU Reading Availible")
        continue

    elif np.abs(pitch) > KILL_ANGLE:
        motors.kill_motors()
        running = False
        break
    
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    pid.set_constants((0.3,0,0.0))
    motor_value = pid.compute(pitch, dt)

    motors.update_motors(motor_value)

    time.sleep(LOOP_DELAY)


    