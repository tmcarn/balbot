from gpiozero import Motor, PWMOutputDevice, OutputDevice

import time

motor = Motor(6, 13)
pwm = PWMOutputDevice(26)

stby = OutputDevice(5)

print("Enabling motor (STBY HIGH)...")
stby.on()  # Enable the motor driver
pwm.on()


pwm.pulse(n=5)

motor.forward()
