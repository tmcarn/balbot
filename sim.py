from pymultiwii import MultiWii
import time
import numpy as np
from matplotlib import pyplot as plt
import math
import pygame
import gymnasium as gym


class Env(gym.Env):

    def __init__(self):
        super().__init__()
        self.screen = pygame.display.set_mode((1000,600))
        self.clock = pygame.time.Clock()
        self.tick = 60
        self.dt = 1/self.tick # 1/60th of a second

        self.wheel = pygame.image.load("assets/small wheel.png")
        self.wheel = pygame.transform.scale(self.wheel, (200, 200))  # Scale the image if needed
       
        # Environment Properties
        self.g = -981.0 # m/s^2
        self.dampener = 0.9

        # Robot Properties
        self.length = 0.3 # meters
        self.arm_mass = 0.5 #kg
        self.arm_inertia = (1/3) * self.arm_mass * (self.length ** 2)

        self.arm_angle = 0.3
        self.arm_angular_acceleration = 0.0  # Initial angular acceleration
        self.arm_angular_velocity = 0.0  # Initial angular velocity
        

        # Wheel Properties
        self.wheel_radius = 0.1 # meters
        self.wheel_mass = 0.1  # kg
        self.wheel_inertia = 0.5 * self.wheel_mass * (self.wheel_radius ** 2)  # Moment of inertia of the wheel
        
        self.wheel_angular_velocity = 0.0  # Angular velocity of the wheel
        self.wheel_angle = 0.0 # Angular position of the wheel
        self.wheel_position = np.array([500, 600 - self.wheel_radius])
        self.wheel_torque = 0.0

        self.base_torque = .1

        self.update_screen()


    def update_properities(self):
        # Angular motion (robot tilt)
        self.arm_angular_acceleration = -(self.arm_mass * self.g * self.length * math.sin(self.arm_angle) - self.wheel_torque) / self.arm_inertia
        self.arm_angular_velocity += self.arm_angular_acceleration * self.dt
        self.arm_angular_velocity *= self.dampener
        self.arm_angle += self.arm_angular_velocity * self.dt # Updated in graphics
        
        # Wheel rotation and linear motion
        self.wheel_angular_acceleration = self.wheel_torque / self.wheel_inertia
        self.wheel_angular_velocity += self.wheel_angular_acceleration * self.dt
        self.wheel_angle -= self.wheel_angular_velocity * self.dt  # Updated in graphics
        self.wheel_position[0] += self.wheel_angular_velocity * self.wheel_radius * self.dt  # Updated in graphics
        
            
    def update_cog_pos(self):
        delta = np.array([math.sin(self.arm_angle) * (self.length / METER_2_PIXEL), math.cos(self.arm_angle) *  (self.length / METER_2_PIXEL)])
        self.cog_pos = self.wheel_position - delta
    
    def update_screen(self):
        # Clear the screen
        self.screen.fill((255, 255, 255))  # White background

        self.update_properities()
        self.update_cog_pos()

        self.fa = 0

        rotated_wheel = pygame.transform.rotate(self.wheel, math.degrees(self.wheel_angle))
        rect = rotated_wheel.get_rect(center=self.wheel_position)
        self.screen.blit(rotated_wheel, rect.topleft)

        pygame.draw.line(self.screen, (255, 0, 0), self.wheel_position, self.cog_pos, 5)

        pygame.display.flip()
        self.clock.tick(self.tick)


    
class PID():

    def __init__(self, kp, ki, kd, setpoint):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.setpoint = setpoint # Angle should be zero for ballanced robot
        
        self.prev_error = 0
        self.integral = 0

        self.dt = 1/60

    def compute(self, current_value):
        error = current_value - self.setpoint

        self.integral += error * self.dt
        der = (error - self.prev_error)/self.dt

        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * der

        self.prev_error = error

        return P + I + D
        


myEnv = Env()
att_controller = PID(10, 10, 5, 0) # Changes torque to achieve desired angle
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Handle key presses for movement and rotation
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        myEnv.wheel_torque = - myEnv.base_torque  # Rotate clockwise
    
    elif keys[pygame.K_RIGHT]:
        myEnv.wheel_torque = myEnv.base_torque  # Rotate clockwise
    
    else:
        myEnv.wheel_torque *= .01
    
    # myEnv.wheel_torque = np.clip(att_controller.compute(myEnv.arm_angle), -1_000_000, 1_000_000)
    myEnv.update_screen()