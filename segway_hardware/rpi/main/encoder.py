from gpiozero import RotaryEncoder
import time

class EncoderAngle:
    def __init__(self, pin_a=23, pin_b=24, ppr=300.8):
        # Create encoder with no max_steps for continuous rotation
        self.encoder = RotaryEncoder(pin_a, pin_b, max_steps=0)
        self.ppr = ppr  # Pulses Per Revolution
        self.prev_steps = 0
        self.prev_time = time.time()
        
    def get_angle(self):
        # Convert steps to degrees
        return 360.0 / self.ppr * self.encoder.steps
        
    def get_rate(self):
        # Calculate angular velocity in degrees/second
        current_time = time.time()
        current_steps = self.encoder.steps
        dt = current_time - self.prev_time
        
        if dt > 0:
            step_diff = current_steps - self.prev_steps
            rate = (step_diff * 360.0 / self.ppr) / dt
            
            self.prev_steps = current_steps
            self.prev_time = current_time
            return rate
        return 0.0

