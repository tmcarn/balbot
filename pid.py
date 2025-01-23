import numpy as np

MOTOR_MAX = 0.1
MOTOR_MIN = 0.05

class PID():
    def __init__(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.setpoint = 0 
        
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = current_value - self.setpoint

        self.integral += error * dt
        der = (error - self.prev_error)/dt

        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * der

        self.prev_error = error

        return np.clip(P + I + D, MOTOR_MIN, MOTOR_MAX)
    
    def set_constants(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd