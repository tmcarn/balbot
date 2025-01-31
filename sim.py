from pymultiwii import MultiWii
import time
import numpy as np
from matplotlib import pyplot as plt
import math
import pygame
from pygame_widgets.slider import Slider
from pygame_widgets.textbox import TextBox
from pygame_widgets import update
import gymnasium as gym

PIXELS_p_M = 200 / 0.065

class Env(gym.Env):

    def __init__(self):
        super().__init__()
        self.screen = pygame.display.set_mode((1000,600))
        self.clock = pygame.time.Clock()
        self.tick = 30
        self.dt = 1/self.tick # 1/60th of a second

        self.p_slider = Slider(self.screen, 100, 50, 100, 20, min=0, max=0.001, step=.0001, initial=0)
        self.i_slider = Slider(self.screen, 100, 100, 100, 20, min=0, max=0.001, step=.0001, initial=0)
        self.d_slider = Slider(self.screen, 100, 150, 100, 20, min=0, max=0.001, step=.0001, initial=0)

        self.pos_slider = Slider(self.screen, 200, 570, 600, 20, min=150, max=850, step=1, initial=500)

        
        #Create textbox to display slider value

        self.target = pygame.image.load("assets/target.png")
        
        self.wheel = pygame.image.load("assets/small wheel.png")
        self.wheel = pygame.transform.scale(self.wheel, (200, 200))  # Scale the image if needed

        self.robot = pygame.image.load("assets/robot.png")
        self.bot_offset = [37, 130]

        # Load background image
        self.background = pygame.image.load("assets/Floor.png")  # Replace with your image file
        self.background = pygame.transform.scale(self.background, (1000, 600))  # Resize to fit screen
       
        # Environment Properties
        self.g = -9.810 # m/s^2
        self.dampener = 0.85

        # Robot Properties
        self.length = 0.1 # meters
        self.arm_mass = 0.5 #kg
        self.arm_inertia = (1/3) * self.arm_mass * (self.length ** 2)

        self.arm_angle = .5
        self.arm_angular_acceleration = 0.0  # Initial angular acceleration
        self.arm_angular_velocity = 0.0  # Initial angular velocity
        

        # Wheel Properties
        self.wheel_radius = 0.0325 # meters
        self.wheel_mass = 0.01  # kg
        self.wheel_inertia = 0.5 * self.wheel_mass * (self.wheel_radius ** 2)  # Moment of inertia of the wheel
        
        self.wheel_angular_velocity = 0.0  # Angular velocity of the wheel
        self.wheel_angle = 0.0 # Angular position of the wheel
        self.wheel_position = np.array([500, 500 - (self.wheel_radius * PIXELS_p_M)])
        self.wheel_torque = 0.0

        self.max_ang = (math.pi/2) + math.asin(self.wheel_radius/self.length)

        self.base_torque = .6

    def reset(self):
        self.arm_angle = 0.5
        self.arm_angular_acceleration = 0.0  # Initial angular acceleration
        self.arm_angular_velocity = 0.0  # Initial angular velocity

        self.wheel_angular_velocity = 0.0  # Angular velocity of the wheel
        self.wheel_angle = 0.0 # Angular position of the wheel
        self.wheel_position = np.array([500, 500 - (self.wheel_radius * PIXELS_p_M)])
        self.wheel_torque = 0.0


    def update_properities(self):
        # Angular motion (robot tilt)
        self.arm_angular_acceleration = -(self.arm_mass * self.g * self.length * math.sin(self.arm_angle) - self.wheel_torque) / self.arm_inertia
        self.arm_angular_velocity += self.arm_angular_acceleration * self.dt
        self.arm_angular_velocity *= self.dampener
        self.arm_angle += self.arm_angular_velocity * self.dt # Updated in graphics
        self.arm_angle = np.clip(self.arm_angle, -self.max_ang, self.max_ang)
        
        # Wheel rotation and linear motion
        self.wheel_angular_acceleration = self.wheel_torque / self.wheel_inertia
        self.wheel_angular_velocity += self.wheel_angular_acceleration * self.dt
        self.wheel_angle -= self.wheel_angular_velocity * self.dt  # Updated in graphics
        self.wheel_position[0] += (self.wheel_angular_velocity * self.wheel_radius * self.dt)   # Updated in graphics
        
        # # Loop wheel back around
        # if self.wheel_position[0] < 0:
        #     self.wheel_position[0] = 1000
        # elif self.wheel_position[0] > 1000:
        #     self.wheel_position[0] = 0
        
        
   
    def update_screen(self):
        # Clear the screen
        self.screen.blit(self.background, (0, 0)) 

        self.update_properities()

        target_rect = self.target.get_rect(center=[self.pos_slider.getValue(), 490])
        self.screen.blit(self.target, target_rect.topleft)

        rotated_wheel = pygame.transform.rotate(self.wheel, (math.degrees(self.wheel_angle))/PIXELS_p_M)
        rect = rotated_wheel.get_rect(center=self.wheel_position)
        self.screen.blit(rotated_wheel, rect.topleft)

        # Compute rotated image
        rotated_robot = pygame.transform.rotate(self.robot, math.degrees(self.arm_angle))  
        rotated_robot_rect = rotated_robot.get_rect(center=self.wheel_position)

        # Compute pivot offset
        pivot_offset = pygame.math.Vector2(self.bot_offset)
        pivot_offset = pivot_offset.rotate(-math.degrees(self.arm_angle))  # Rotate offset vector

        # Adjust image position based on pivot rotation
        rotated_robot_rect.center -= pivot_offset

        # Draw rotated image
        self.screen.blit(rotated_robot, rotated_robot_rect.topleft)

        
        update(pygame.event.get())
        pygame.display.flip()
        self.clock.tick(self.tick)


    
class PID():

    def __init__(self, kp, ki, kd, setpoint, deadband):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.setpoint = setpoint # Angle should be zero for ballanced robot
        self.deadband = deadband
        self.MAX_INT = 1
        
        self.prev_error = 0
        self.integral = 0
        self.der = 0

        self.dt = 1/30

    def set_k_vals(self, values):
        self.Kp, self.Ki, self.Kd = values

    def compute(self, current_value):
        error = self.setpoint - current_value 

        self.integral += error * self.dt
        self.integral = np.clip(self.integral, -self.MAX_INT, self.MAX_INT)
        self.der = (error - self.prev_error)/self.dt

        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * self.der

        self.prev_error = error

        return (P + I + D)
        


myEnv = Env()
att_controller = PID(1, 0, 0.05, 0, math.radians(1)) # Changes torque to achieve desired angle
pos_controller = PID(0.001, 0, 0, 800, 10) # Changes desired angle to get desired location
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Handle key presses for movement and rotation
    keys = pygame.key.get_pressed()

    if keys[pygame.K_r]:
        myEnv.reset()
    # if keys[pygame.K_LEFT]:
    #     myEnv.wheel_torque = - myEnv.base_torque  # Rotate clockwise
    
    # elif keys[pygame.K_RIGHT]:
    #     myEnv.wheel_torque = myEnv.base_torque  # Rotate clockwise
    
    # else:
    #     myEnv.wheel_torque *= .01
    # Get slider value


    # pos_controller.setpoint = myEnv.slider.getValue()

    p_val = myEnv.p_slider.getValue()
    i_val = myEnv.i_slider.getValue()
    d_val = myEnv.d_slider.getValue()

    pos_controller.set_k_vals((p_val, i_val, d_val))
    pos_controller.setpoint = myEnv.pos_slider.getValue()

    att_controller.setpoint = -1 * np.clip(pos_controller.compute(myEnv.wheel_position[0]), -0.05, 0.05)

    print(f"ANGLE SET TO: {att_controller.setpoint:.2f} degrees; DERIVATIVE TERM: {att_controller.der:.2f}")
    
    myEnv.wheel_torque = np.clip(att_controller.compute(myEnv.arm_angle), -myEnv.base_torque, myEnv.base_torque)
    
    myEnv.update_screen()