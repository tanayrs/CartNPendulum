from gpiozero import RotaryEncoder
import numpy as np
import time
import collections

class EncoderAngle:
    def __init__(self, pin_a=23, pin_b=24, ppr=2262, wheel_radius=0.1, filter_size=10):
        # Create encoder with no max_steps for continuous rotation
        self.encoder = RotaryEncoder(pin_a, pin_b, max_steps=0)
        self.ppr = ppr  # Pulses Per Revolution
        self.wheel_radius = wheel_radius
        
        # For rate calculation
        self.prev_steps = 0
        self.prev_time = time.time()
        
        # For filtering
        self.filter_size = filter_size
        self.rate_buffer = collections.deque(maxlen=filter_size)
        self.vel_buffer = collections.deque(maxlen=filter_size)
        
        # Initialize with zeros
        for _ in range(filter_size):
            self.rate_buffer.append(0.0)
            self.vel_buffer.append(0.0)

    def get_angle(self):
        # Convert steps to radians
        return 2 * np.pi / self.ppr * self.encoder.steps

    def get_rate(self):
        # Calculate angular velocity in rad/second
        current_time = time.time()
        current_steps = self.encoder.steps
        dt = current_time - self.prev_time
        
        if dt > 0.001:  # Prevent division by very small numbers
            step_diff = current_steps - self.prev_steps
            raw_rate = (step_diff * 2 * np.pi / self.ppr) / dt
            
            # Add to buffer for filtering
            self.rate_buffer.append(raw_rate)
            
            # Update stored values
            self.prev_steps = current_steps
            self.prev_time = current_time
            
            # Return filtered rate (moving average)
            return sum(self.rate_buffer) / self.filter_size
        
        return 0.0

    def get_pos(self):
        # Calculate linear position from encoder steps
        rotations = self.encoder.steps / self.ppr  # Fixed the self-ppr typo
        perimeter = 2 * np.pi * self.wheel_radius
        return rotations * perimeter

    def get_vel(self):
        # Calculate linear velocity from angular rate
        rate = self.get_rate()
        vel = rate * self.wheel_radius
        
        # Add to buffer for filtering
        self.vel_buffer.append(vel)
        
        # Return filtered velocity
        return sum(self.vel_buffer) / self.filter_size

