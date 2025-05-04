'''
Quadrature Encoder Interface

This module provides classes for interfacing with a quadrature encoder to measure position and velocity of the segway
It implements both low-level hardware communication and high-level signal processing.

Features:
- Real-time quadrature decoding using hardware interrupts via pigpio
- High-precision position tracking with proper quadrature state transition handling
- Velocity calculation with configurable moving average filter for noise reduction
- Conversion between angular and linear measurements (radians, meters, rad/s, m/s)
- Resource management with clean initialization and shutdown procedures

Key Components:
- PiEncoder: Low-level class that interfaces with GPIO pins and implements
  the quadrature decoding state machine using edge detection callbacks
- EncoderProcessor: High-level class that converts raw encoder ticks to
  meaningful position and velocity measurements with filtering

Usage:
    # Initialize hardware interface
    encoder_hw = PiEncoder(pin_a=23, pin_b=24)
    
    # Initialize processor (for a 600 PPR encoder with 10cm wheel radius)
    processor = EncoderProcessor(pulses_per_rev=600, wheel_rad=0.1)
    
    # In main loop
    while running:
        # Get latest encoder reading and update processor
        processor.update(encoder_hw.read())
        
        # Get position and velocity measurements
        cart_position = processor.get_position_meters()
        cart_velocity = processor.get_speed_ms()
'''

# imports
import pigpio
import time
import math
import numpy as np
from collections import deque

class PiEncoder:
    def __init__(self, pin_a, pin_b):
        self.pi = pigpio.pi()              # Initialize pigpio interface
        self.pin_a = pin_a                 # Store GPIO pin for A channel
        self.pin_b = pin_b                 # Store GPIO pin for B channel
        self.position = 0                  # Track encoder position (tick count)
        self.state = 0                     # Current state of encoder signals
        
        # Enable internal pull-up resistors on encoder pins
        self.pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Read initial encoder pin states
        self.last_a = self.pi.read(pin_a)
        self.last_b = self.pi.read(pin_b)
        
        # Register edge detection callbacks for both pins
        self.cb_a = self.pi.callback(pin_a, pigpio.EITHER_EDGE, self._update)
        self.cb_b = self.pi.callback(pin_b, pigpio.EITHER_EDGE, self._update)
        
    def _update(self, gpio, level, tick):
        # Read current state of both encoder pins
        a = self.pi.read(self.pin_a)
        b = self.pi.read(self.pin_b)
        
        # Create 4-bit state from previous and current pin readings
        state = (self.last_a << 1) | self.last_b
        state |= (a << 3) | (b << 2)
        
        # Quadrature decoding lookup table for determining direction
        # Maps 16 possible state transitions to direction changes
        transition_table = {
            0b0000: 0, 0b0001: 1, 0b0010: -1, 0b0011: 2,
            0b0100: -1, 0b0101: 0, 0b0110: -2, 0b0111: 1,
            0b1000: 1, 0b1001: -2, 0b1010: 0, 0b1011: -1,
            0b1100: 2, 0b1101: -1, 0b1110: 1, 0b1111: 0
        }
        
        # Update position based on transition
        self.position += transition_table.get(state, 0)
        
        # Save current pin states for next update
        self.last_a = a
        self.last_b = b
        
    def read(self):
        # Return current position count
        return self.position
        
    def read_and_reset(self):
        # Get current position and reset counter to zero
        pos = self.position
        self.position = 0
        return pos
        
    def cleanup(self):
        # Cancel callbacks and release resources
        self.cb_a.cancel()
        self.cb_b.cancel()
        self.pi.stop()

class EncoderProcessor:
    def __init__(self, pulses_per_rev, window_size=8, wheel_rad = 0.1, cutoff_freq=1.0):
        # Number of ticks per full revolution (4x for quadrature encoding)
        self.ticks_per_rev = 4 * pulses_per_rev
        self.cutoff_freq = cutoff_freq    # Low-pass filter cutoff frequency (not used in current implementation)
        self.window_size = window_size    # Size of moving average window for speed filtering
        
        self.last_ticks = 0               # Store previous tick count
        self.position = 0                 # Current angular position in radians
        self.raw_speed = 0.0              # Unfiltered angular velocity
        self.wheel_rad = wheel_rad        # Wheel radius in meters (for conversion to linear distance/speed)
        self.speed = 0.0                  # Filtered angular velocity
        self.last_time = None             # Timestamp of last measurement
        
        # Create a deque to store the last window_size velocity measurements for moving average filter
        self.velocity_buffer = deque(maxlen=window_size)
        
    def update(self, raw_ticks):
        # Main update method called with new encoder reading
        self._update_wheel(raw_ticks)     # Update position
        self._calculate_speed(raw_ticks)  # Calculate and filter speed
        
    def _update_wheel(self, ticks):
        # Convert encoder ticks to angular position in radians
        self.position = (ticks / self.ticks_per_rev) * 2 * np.pi 
        
    def _calculate_speed(self, current_ticks):
        current_time = time.perf_counter()  # Get current time with high precision
        if self.last_time is None:
            # Initialize on first call
            self.last_time = current_time
            self.last_ticks = current_ticks
            self.raw_speed = 0.0
            # Initialize buffer with zeros
            for _ in range(self.window_size):
                self.velocity_buffer.append(0.0)
            return
            
        delta_ticks = current_ticks - self.last_ticks  # Change in encoder ticks
        delta_time = current_time - self.last_time     # Time elapsed since last measurement
        
        if delta_time > 0:
            # Calculate raw velocity in radians per second
            raw_velocity = (delta_ticks * 2 * np.pi/ self.ticks_per_rev) / delta_time
            self.raw_speed = raw_velocity
            
            # Add to moving average buffer
            self.velocity_buffer.append(raw_velocity)
            
            # Calculate filtered speed (simple moving average)
            self.speed = sum(self.velocity_buffer) / len(self.velocity_buffer)
        
        # Store current values for next calculation
        self.last_ticks = current_ticks
        self.last_time = current_time
        
    def get_position_rad(self):
        # Return angular position in radians
        return self.position

    def get_position_meters(self):
        # Convert angular position to linear distance using wheel radius
        return self.position*self.wheel_rad
        
    def get_speed(self):
        # Return filtered angular velocity in radians per second
        return self.speed

    def get_speed_ms(self):
        # Convert angular velocity to linear velocity (m/s) using wheel radius
        return self.speed*self.wheel_rad
        
    def get_raw_speed(self):
        # Returns the unfiltered speed value (radians per second)
        return self.raw_speed