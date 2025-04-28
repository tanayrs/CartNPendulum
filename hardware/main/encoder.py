# Class to read x and xdot from Encoder

'''
Quadrature Encoder Interface

This module provides a comprehensive interface for reading and processing data from a quadrature encoder, used to measure the cart position (x) and velocity (x_dot) in 
the Cart-Pendulum reinforcement learning platform.

Features:
- Interrupt-driven quadrature decoding using pigpio library
- High-precision position tracking
- Velocity calculation with configurable moving average filter
- Support for multiple units (radians, meters, radians/sec, meters/sec)
- Efficient state transition handling for reliable pulse counting

Components:
- PiEncoder: Low-level class that interfaces directly with GPIO pins
  and implements the quadrature decoding state machine
- EncoderProcessor: High-level class that converts raw encoder ticks
  to meaningful position and velocity measurements

Usage:
    # Initialize hardware interface
    encoder_hw = PiEncoder(pin_a=23, pin_b=24)
    
    # Initialize processor (for a 600 PPR encoder with 100mm wheel)
    processor = EncoderProcessor(pulses_per_rev=600, wheel_rad=0.1)
    
    # In main control loop
    while running:
        # Update processor with latest encoder reading
        processor.update(encoder_hw.read())
        
        # Get cart measurements
        x = processor.get_position_meters()
        x_dot = processor.get_speed_ms()
        
        # Use these values for control or state estimation
        
    # Cleanup when done
    encoder_hw.cleanup()

Dependencies:
    - pigpio: For GPIO access and interrupt handling
    - numpy: For mathematical operations
    - collections.deque: For efficient moving average calculation
'''

# imports
import pigpio
import time
import math
import numpy as np
from collections import deque

class PiEncoder:
    def __init__(self, pin_a, pin_b):
        # Connect to pigpio daemon
        self.pi = pigpio.pi()
        # GPIO pins for encoder channels
        self.pin_a = pin_a
        self.pin_b = pin_b
        # Counter for encoder ticks
        self.position = 0
        # Current state in state machine
        self.state = 0
        
        # Enable internal pull-up resistors on encoder pins
        self.pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Read initial pin states
        self.last_a = self.pi.read(pin_a)
        self.last_b = self.pi.read(pin_b)
        
        # Register edge-triggered callbacks for both encoder channels
        self.cb_a = self.pi.callback(pin_a, pigpio.EITHER_EDGE, self._update)
        self.cb_b = self.pi.callback(pin_b, pigpio.EITHER_EDGE, self._update)
        
    def _update(self, gpio, level, tick):
        # Read current pin states
        a = self.pi.read(self.pin_a)
        b = self.pi.read(self.pin_b)
        # Encode previous and current states into a 4-bit value
        state = (self.last_a << 1) | self.last_b
        state |= (a << 3) | (b << 2)
        
        # Lookup table for quadrature state transitions:
        # 0: No movement
        # 1/-1: Forward/backward by one step
        # 2/-2: Forward/backward by two steps (missed a state)
        transition_table = {
            0b0000: 0, 0b0001: 1, 0b0010: -1, 0b0011: 2,
            0b0100: -1, 0b0101: 0, 0b0110: -2, 0b0111: 1,
            0b1000: 1, 0b1001: -2, 0b1010: 0, 0b1011: -1,
            0b1100: 2, 0b1101: -1, 0b1110: 1, 0b1111: 0
        }
        
        # Update position based on state transition
        self.position += transition_table.get(state, 0)
        # Store current states for next update
        self.last_a = a
        self.last_b = b
        
    def read(self):
        # Return current position
        return self.position
        
    def read_and_reset(self):
        # Get current position and reset counter
        pos = self.position
        self.position = 0
        return pos
        
    def cleanup(self):
        # Cancel callbacks and disconnect from pigpio
        self.cb_a.cancel()
        self.cb_b.cancel()
        self.pi.stop()

class EncoderProcessor:
    def __init__(self, pulses_per_rev, window_size=8, wheel_rad=0.1, cutoff_freq=1.0):
        # Calculate total ticks per revolution (4x for quadrature encoding)
        self.ticks_per_rev = 4 * pulses_per_rev
        self.cutoff_freq = cutoff_freq  # Low-pass filter cutoff frequency
        self.window_size = window_size  # Size of moving average window
        
        self.last_ticks = 0            # Previous encoder tick count
        self.position = 0              # Current position in radians
        self.raw_speed = 0.0           # Unfiltered angular velocity
        self.wheel_rad = wheel_rad     # Wheel radius in meters
        self.speed = 0.0               # Filtered angular velocity
        self.last_time = None          # Timestamp of previous measurement
        
        # Create a deque to store the last window_size velocity measurements
        self.velocity_buffer = deque(maxlen=window_size)
        
    def update(self, raw_ticks):
        # Process new encoder reading
        self._update_wheel(raw_ticks)
        self._calculate_speed(raw_ticks)
        
    
    def _update_wheel(self, ticks):
        # Convert encoder ticks to angular position in radians
        self.position = (ticks / self.ticks_per_rev) * 2 * np.pi 
        
    def _calculate_speed(self, current_ticks):
        current_time = time.perf_counter()  # Get current timestamp
        if self.last_time is None:
            # Initialize on first call
            self.last_time = current_time
            self.last_ticks = current_ticks
            self.raw_speed = 0.0
            # Initialize buffer with zeros
            for _ in range(self.window_size):
                self.velocity_buffer.append(0.0)
            return
            
        delta_ticks = current_ticks - self.last_ticks  # Change in ticks
        delta_time = current_time - self.last_time     # Time elapsed
        
        if delta_time > 0:
            # Calculate raw velocity in radians/sec
            raw_velocity = (delta_ticks * 2 * np.pi / self.ticks_per_rev) / delta_time
            self.raw_speed = raw_velocity
            
            # Add to moving average buffer
            self.velocity_buffer.append(raw_velocity)
            
            # Calculate filtered speed (average of buffer)
            self.speed = sum(self.velocity_buffer) / len(self.velocity_buffer)
        
        # Update previous values for next iteration
        self.last_ticks = current_ticks
        self.last_time = current_time
        
    def get_position_rad(self):
        # Return position in radians
        return self.position

    def get_position_meters(self):
        # Convert angular position to linear position using wheel radius
        return self.position * self.wheel_rad
        
    def get_speed(self):
        # Return angular velocity in radians/sec
        return self.speed

    def get_speed_ms(self):
        # Convert angular velocity to linear velocity in meters/sec
        return self.speed * self.wheel_rad
        
    def get_raw_speed(self):
        # Returns the unfiltered speed value
        return self.raw_speed