from gpiozero import PWMOutputDevice
import time
import numpy as np

# ESC configuration
PWM_FREQUENCY = 50  # 50 Hz (20ms period)

GEAR_REDUCTION = 21.3
ENCODER_STEPS = 11

class MotorController():

    def __init__(self, pin) -> None:
       pass