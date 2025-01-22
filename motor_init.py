from gpiozero import PWMOutputDevice
import time

# Pin configuration
PIN = 26

# ESC configuration
PWM_FREQUENCY = 50  # 50 Hz (20ms period)
MIN_THROTTLE = 0.05  # 1ms pulse width (5% duty cycle)
MAX_THROTTLE = 0.10  # 2ms pulse width (10% duty cycle)
MID_THROTTLE = (MIN_THROTTLE + MAX_THROTTLE) / 2  # Midpoint

# Initialize motor
motor = PWMOutputDevice(PIN, frequency=PWM_FREQUENCY)

# Arming sequence
print("Arming ESC...")
motor.value = MAX_THROTTLE  # Full throttle (2ms pulse)
time.sleep(2)  # Hold for 2 seconds

motor.value = MIN_THROTTLE  # Zero throttle (1ms pulse)
time.sleep(2)  # Hold for 2 seconds

print("ESC armed. Ready to control!")

# Example throttle levels
try:
    print("Half throttle")
    motor.value = MID_THROTTLE  # Mid throttle (1.5ms pulse)
    time.sleep(4)

    print("Full throttle")
    motor.value = MAX_THROTTLE  # Full throttle (2ms pulse)
    time.sleep(2)

    print("Zero throttle")
    motor.value = MIN_THROTTLE  # Zero throttle (1ms pulse)
    time.sleep(2)

finally:
    # Ensure motor is stopped
    motor.value = 0
    print("Motor stopped. All done!")
