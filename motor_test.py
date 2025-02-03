from gpiozero import Motor, PWMOutputDevice, OutputDevice

import time

motor = Motor(6, 13)
pwm = PWMOutputDevice(26)

stby = OutputDevice(5)

print("Enabling motor (STBY HIGH)...")
stby.on()  # Enable the motor driver
pwm.on()


print("Forward")
motor.forward()
time.sleep(2)

print("Backwards")
motor.backward()
time.sleep(2)