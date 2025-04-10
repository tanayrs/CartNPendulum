from gpiozero import RotaryEncoder
import numpy as np
import time

class EncoderAngle:
    def __init__(self, pin_a=23, pin_b=24, ppr=2262, wheel_radius = 0.1):
        # Create encoder with no max_steps for continuous rotation
        self.encoder = RotaryEncoder(pin_a, pin_b, max_steps=0)
        self.ppr = ppr  # Pulses Per Revolution
        self.wheel_radius = wheel_radius
        self.prev_steps = 0
        self.prev_time = time.time()
        self.angle = 0
        self.angle_rate = 0
        self.pos = 0
        self.vel = 0
        
    def get_angle(self):
        # Convert steps to radians 
        return 2 * np.pi / self.ppr * self.encoder.steps
        
    def get_rate(self):
        # Calculate angular velocity in rad/second
        current_time = time.time()
        current_steps = self.encoder.steps
        dt = current_time - self.prev_time
        
        if dt > 0:
            step_diff = current_steps - self.prev_steps
            rate = (step_diff * 2 * np.pi / self.ppr) / dt
            
            self.prev_steps = current_steps
            self.prev_time = current_time
            return rate
        return 0.0

    def get_pos(self):
        rotations = 1 / self.ppr * self.encoder.steps
        perimeter = 2 * np.pi * self.wheel_radius
        self.pos = rotations * perimeter
        return self.pos

    def get_vel(self):
        return self.get_rate() * self.wheel_radius
