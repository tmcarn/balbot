from gpiozero import Motor
from gpiozero import PWMOutputDevice

import time

motor = Motor(6, 13)
pwm = PWMOutputDevice(26)

motor.forward(0.5)
time.sleep(2)

motor.backward()
time.sleep(2)