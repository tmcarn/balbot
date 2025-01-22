from gpiozero import PWMOutputDevice
import time

PIN = 26

motor = PWMOutputDevice(PIN, frequency=50)

def set_motor_speed(duty_cycle): 
    motor.value = duty_cycle / 1000
    print(f'Motor speed set to {duty_cycle}%')


try:
    for i in range(100, 50, -1):
        set_motor_speed(i)
        time.sleep(0.2)

    for i in range (50, 100):
        set_motor_speed(i)
        time.sleep(0.2)

    time.sleep(5)

except KeyboardInterrupt:
    pass

finally:
    motor.value = 0
