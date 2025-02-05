import numpy as np
import math

PWM_MAX = 1

class PID():
    def __init__(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.setpoint = 0

        self.alpha = 0.95
        self.max_windup = 1
        
        self.prev_error = 0
        self.integral = 0
        self.derivative = 0

    def compute(self, current_value, dt):
        error = current_value - self.setpoint

        self.integral += error * dt
        self.integral = np.clip(self.integral, -PWM_MAX / self.Ki if self.Ki != 0 else 0, PWM_MAX / self.Ki if self.Ki != 0 else 0)

        # Smoothing
        raw_derivative = (error - self.prev_error) / dt
        self.derivative = (self.alpha * self.derivative) + ((1 - self.alpha) * raw_derivative)  # 95% of previous derivative + 5% of current derivative

        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * self.derivative

        self.prev_error = error

        motor_value = (P + I + D)
        print(f'Raw PWM Value: {motor_value}')
        motor_value = np.clip(motor_value, -PWM_MAX, PWM_MAX)

        return motor_value
    
    def set_constants(self, values):
        self.Kp, self.Ki, self.Kd = values