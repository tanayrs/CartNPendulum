from gpiozero import RotaryEncoder
import numpy as np
import time
import collections

class EncoderAngle:
    def __init__(self, pin_a=23, pin_b=24, ppr=2262, wheel_radius=0.1, filter_size=10):
        """
        Initialize the encoder with filtering and rate calculation.
        """
        try:
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

            # Initialize buffers with zeros
            for _ in range(filter_size):
                self.rate_buffer.append(0.0)
                self.vel_buffer.append(0.0)
        except Exception as e:
            print(f"Error initializing encoder: {e}")

    def get_angle(self):
        """
        Get the angular position in radians based on encoder steps.
        """
        try:
            return 2 * np.pi / self.ppr * self.encoder.steps  # Convert steps to radians
        except Exception as e:
            print(f"Error reading angle: {e}")
            return 0.0

    def get_rate(self):
        """
        Calculate angular velocity in rad/s using filtered data.
        """
        try:
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

                return sum(self.rate_buffer) / len(self.rate_buffer)  # Return filtered rate

        except Exception as e:
            print(f"Error calculating rate: {e}")

        return 0.0

    def get_pos(self):
        """
        Calculate linear position based on encoder steps.
        """
        try:
            rotations = self.encoder.steps / self.ppr  # Calculate rotations from steps
            perimeter = 2 * np.pi * self.wheel_radius  # Wheel perimeter (circumference)

            return rotations * perimeter  # Position in meters

        except Exception as e:
            print(f"Error calculating position: {e}")

        return 0.0

    def get_vel(self):
        """
        Calculate linear velocity in m/s using filtered angular velocity.
        """
        try:
            rate = self.get_rate()  # Angular velocity in rad/s
            vel = rate * self.wheel_radius  # Linear velocity in m/s

            # Add to buffer for filtering
            self.vel_buffer.append(vel)

            # Return filtered velocity (moving average)
            return sum(self.vel_buffer) / len(self.vel_buffer)

        except Exception as e:
            print(f"Error calculating velocity: {e}")

        return 0.0
