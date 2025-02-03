from gpiozero import Motor, PWMOutputDevice, OutputDevice

import time
import numpy as np


GEAR_REDUCTION = 21.3
ENCODER_STEPS = 11

STANDBY = 5

# Left Motor
L_FORWARD = 27
L_BACKWARD = 22
L_PWM = 17

# Right Motor
R_FORWARD = 6
R_BACKWARD = 13
R_PWM = 26

class MotorController():

    def __init__(self) -> None:
       self.stby = OutputDevice(STANDBY)
       self.stby.on() # Motor Driver Active

       self.motor_l = Motor(L_FORWARD, L_BACKWARD)
       self.pwm_l = PWMOutputDevice(L_PWM)

       self.motor_r = Motor(R_FORWARD, R_BACKWARD)
       self.pwm_r = PWMOutputDevice(R_PWM)

       print("Motors Initialized")

    def kill_motors(self):
        self.motor_l.stop()
        self.motor_r.stop()
        self.stby.off()

    def update_motors(self, pwm_value):

        if pwm_value > 0: 
            self.pwm_l.value = pwm_value
            self.pwm_r.value = pwm_value

            self.motor_l.forward()
            self.motor_r.forward()

        else:
            pwm_value = np.abs(pwm_value)
            self.pwm_l.value = pwm_value
            self.pwm_r.value = pwm_value

            self.motor_l.backward()
            self.motor_r.backward()


