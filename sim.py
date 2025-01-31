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
        pygame.init()
        self.screen = pygame.display.set_mode((1000,600))
        pygame.display.set_caption("PID Demonstration")

        self.clock = pygame.time.Clock()
        self.tick = 30
        self.dt = 1/self.tick # 1/60th of a second

        self.p_slider = Slider(self.screen, 250 - 50, 50, 100, 20, min=0, max=0.001, step=.00001, initial=0)
        self.i_slider = Slider(self.screen, 500 - 50, 50, 100, 20, min=0, max=0.001, step=.00001, initial=0)
        self.d_slider = Slider(self.screen, 750 - 50, 50, 100, 20, min=0, max=0.001, step=.00001, initial=0)

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
        self.wheel_position[0] += (self.wheel_angular_velocity * self.wheel_radius * self.dt)

        drift_rate = 1  # Small constant drift
        self.wheel_position[0] += drift_rate * self.dt

        
        
        
   
    def update_screen(self):
        # Clear the screen
        self.screen.blit(self.background, (0, 0))

        # Define a font and size
        big_font = pygame.font.Font(None, 36)  # None uses the default font
        small_font = pygame.font.Font(None, 24)

        pid_text = big_font.render("PID Controls", True, (255, 255, 255))  # White text
        text_rect = pid_text.get_rect(center=(500, 25))  # Center the text
        self.screen.blit(pid_text, text_rect)

        p_text = small_font.render(f"K_p = {(self.p_slider.getValue()*1000):.2f}", True, (255,255,255))
        p_rect = p_text.get_rect(center = (250, 100))
        self.screen.blit(p_text, p_rect)

        i_text = small_font.render(f"K_i = {(self.i_slider.getValue()*1000):.2f}", True, (255,255,255))
        i_rect = i_text.get_rect(center = (500, 100))
        self.screen.blit(i_text, i_rect)

        d_text = small_font.render(f"K_d = {(self.d_slider.getValue()*1000):.2f}", True, (255,255,255))
        d_rect = d_text.get_rect(center = (750, 100))
        self.screen.blit(d_text, d_rect)


        reset_text = small_font.render("Click 'R' to Reset", True, (255,255,255))
        reset_rect = reset_text.get_rect(center = (900, 580))
        self.screen.blit(reset_text, reset_rect)

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

    def __init__(self, kp, ki, kd, setpoint):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.setpoint = setpoint # Angle should be zero for ballanced robot
        self.MAX_INT = 100
        
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
att_controller = PID(1, 0, 0.05, 0) # Changes torque to achieve desired angle
pos_controller = PID(0.001, 0, 0, 800) # Changes desired angle to get desired location
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Handle key presses for movement and rotation
    keys = pygame.key.get_pressed()

    if keys[pygame.K_r]:
        myEnv.reset()

    p_val = myEnv.p_slider.getValue()
    i_val = myEnv.i_slider.getValue()
    d_val = myEnv.d_slider.getValue()

    pos_controller.set_k_vals((p_val, i_val, d_val))
    pos_controller.setpoint = myEnv.pos_slider.getValue()

    measured_position = myEnv.wheel_position[0]
    att_controller.setpoint = -1 * np.clip(pos_controller.compute(measured_position), -0.05, 0.05)
    
    myEnv.wheel_torque = np.clip(att_controller.compute(myEnv.arm_angle), -myEnv.base_torque, myEnv.base_torque) + 0.02
    
    myEnv.update_screen()