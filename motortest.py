from gpiozero import PWMOutputDevice

import time

PIN = 26

motor = PWMOutputDevice(PIN, frequency=50)

# Full throttle
print("Full throttle")
motor.value = 0.1
time.sleep(2)

# Low throttle
print("Low throttle")
motor.value = 0.05
time.sleep(2)

# Half throttle
print("Half throttle")
motor.value = 0.075
time.sleep(4)

motor.value = 0
print("All done")