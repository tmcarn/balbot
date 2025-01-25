from gpiozero import PWMOutputDevice
import time
import numpy as np

# ESC configuration
PWM_FREQUENCY = 50  # 50 Hz (20ms period)
MIN_THROTTLE = 0.05  # 1ms pulse width (5% duty cycle)
MAX_THROTTLE = 0.10  # 2ms pulse width (10% duty cycle)
MID_THROTTLE = (MIN_THROTTLE + MAX_THROTTLE) / 2  # Midpoint

class MotorController():

    def __init__(self, pin) -> None:
        # Pin configuration
        PIN = pin

        # Initialize motor
        self.motor = PWMOutputDevice(PIN, frequency=PWM_FREQUENCY)

        # Arming sequence
        print("Arming ESC...")
        self.motor.value = MAX_THROTTLE  # Full throttle (2ms pulse)
        time.sleep(2)  # Hold for 2 seconds

        self.motor.value = MIN_THROTTLE  # Zero throttle (1ms pulse)
        time.sleep(2)  # Hold for 2 seconds

        print("ESC armed. Ready to control!")

    def set_motor_speed(self, value):
        self.motor.value = value

    def kill_motor(self):
        self.motor.value = 0

    def test_motor(self):
        try:
            print("Ramp Up")
            up_range = np.linspace(MIN_THROTTLE, MAX_THROTTLE, num=50)
            for i in up_range:
                self.motor.value = i
                time.sleep(.5)

            print("Ramp Down")
            down_range = np.linspace(MAX_THROTTLE, MIN_THROTTLE, num=50)
            for i in down_range:
                self.motor.value = i
                time.sleep(.5)

        finally:
            # Ensure motor is stopped
            self.motor.value = 0
            print("Motor stopped. All done!")

        

# Example throttle levels
