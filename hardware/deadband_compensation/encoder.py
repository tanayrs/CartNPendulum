'''
Quadrature Encoder Interface for Motor Deadband Characterization

This module provides classes for interfacing with a quadrature encoder to measure 
position and angular velocity of a motor, specifically for characterizing the motor 
deadband. The deadband is the range of input values where 
the motor doesn't respond due to static friction and other mechanical factors.

Features:
- Real-time quadrature decoding using pigpio edge detection callbacks
- High-precision position tracking with proper state transition handling
- Velocity calculation with configurable moving average filter for noise reduction
- Angular position measurement in degrees
- Raw and filtered velocity outputs for comparison and analysis

Components:
- PiEncoder: Low-level class that interfaces directly with GPIO pins
  and implements the quadrature decoding state machine using edge detection
- EncoderProcessor: High-level class that converts raw encoder ticks to
  meaningful position and velocity measurements with filtering

Usage:
    # Initialize hardware interface
    encoder_hw = PiEncoder(pin_a=23, pin_b=24)
    
    # Initialize processor (for a 600 PPR encoder)
    processor = EncoderProcessor(pulses_per_rev=600, window_size=8)
    
    # In main loop for deadband testing
    while running:
        # Update processor with latest encoder reading
        processor.update(encoder_hw.read())
        
        # Get position and velocity measurements
        position = processor.get_position_degrees()
        velocity = processor.get_speed()
        
        # Use these values to determine motor deadband
        # (when motor input increases but velocity remains zero)
'''

# imports
import pigpio
import time
from collections import deque

class PiEncoder:
    def __init__(self, pin_a, pin_b):
        self.pi = pigpio.pi()                # Initialize pigpio interface
        self.pin_a = pin_a                   # GPIO pin for encoder channel A
        self.pin_b = pin_b                   # GPIO pin for encoder channel B
        self.position = 0                    # Current encoder position (tick count)
        self.state = 0                       # Current state of the quadrature decoder
        
        # Enable pull-up resistors on encoder pins to prevent floating inputs
        self.pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Read initial state of encoder pins
        self.last_a = self.pi.read(pin_a)    # Last known state of pin A
        self.last_b = self.pi.read(pin_b)    # Last known state of pin B
        
        # Register callback functions for edge detection on both pins
        self.cb_a = self.pi.callback(pin_a, pigpio.EITHER_EDGE, self._update)
        self.cb_b = self.pi.callback(pin_b, pigpio.EITHER_EDGE, self._update)
        
    def _update(self, gpio, level, tick):
        # Read current state of both encoder pins
        a = self.pi.read(self.pin_a)
        b = self.pi.read(self.pin_b)
        
        # Construct a 4-bit state using previous and current pin states
        # Format: [prev_A, prev_B, curr_A, curr_B]
        state = (self.last_a << 1) | self.last_b
        state |= (a << 3) | (b << 2)
        
        # Lookup table for quadrature state transitions:
        # 0: No change, 1: Forward step, -1: Backward step
        # 2/-2: Error correction for missed steps
        transition_table = {
            0b0000: 0, 0b0001: 1, 0b0010: -1, 0b0011: 2,
            0b0100: -1, 0b0101: 0, 0b0110: -2, 0b0111: 1,
            0b1000: 1, 0b1001: -2, 0b1010: 0, 0b1011: -1,
            0b1100: 2, 0b1101: -1, 0b1110: 1, 0b1111: 0
        }
        
        # Update position counter based on state transition
        self.position += transition_table.get(state, 0)
        
        # Store current pin states for next update
        self.last_a = a
        self.last_b = b
        
    def read(self):
        # Return current encoder position
        return self.position
        
    def read_and_reset(self):
        # Return current position and reset counter to zero
        pos = self.position
        self.position = 0
        return pos
        
    def cleanup(self):
        # Cancel callbacks and release GPIO resources
        self.cb_a.cancel()
        self.cb_b.cancel()
        self.pi.stop()

class EncoderProcessor:
    def __init__(self, pulses_per_rev, window_size=8, cutoff_freq=1.0):
        # Calculate total encoder ticks per revolution (4x for quadrature encoding)
        self.ticks_per_rev = 4 * pulses_per_rev
        self.cutoff_freq = cutoff_freq  # For potential future filtering implementations
        self.window_size = window_size  # Size of moving average window for velocity
        
        # Initialize state variables
        self.last_ticks = 0             # Previous encoder tick count
        self.position = 0               # Current angular position in degrees
        self.raw_speed = 0.0            # Unfiltered angular velocity
        self.speed = 0.0                # Filtered angular velocity
        self.last_time = None           # Timestamp of previous measurement
        
        # Create a fixed-size buffer for velocity measurements (moving average filter)
        self.velocity_buffer = deque(maxlen=window_size)
        
    def update(self, raw_ticks):
        # Process new encoder reading
        self._update_wheel(raw_ticks)   # Update position
        self._calculate_speed(raw_ticks)  # Update velocity
        
    def _update_wheel(self, ticks):
        # Convert encoder ticks to angular position in degrees
        self.position = (ticks / self.ticks_per_rev) * 360
        
    def _calculate_speed(self, current_ticks):
        current_time = time.perf_counter()  # Get current timestamp
        if self.last_time is None:
            # First call initialization
            self.last_time = current_time
            self.last_ticks = current_ticks
            self.raw_speed = 0.0
            # Initialize velocity buffer with zeros
            for _ in range(self.window_size):
                self.velocity_buffer.append(0.0)
            return
            
        # Calculate change in ticks and time
        delta_ticks = current_ticks - self.last_ticks
        delta_time = current_time - self.last_time
        
        if delta_time > 0:
            # Calculate instantaneous velocity in degrees per second
            raw_velocity = (delta_ticks * 360 / self.ticks_per_rev) / delta_time
            self.raw_speed = raw_velocity
            
            # Add to moving average buffer
            self.velocity_buffer.append(raw_velocity)
            
            # Calculate filtered speed (simple moving average)
            self.speed = sum(self.velocity_buffer) / len(self.velocity_buffer)
        
        # Update state for next calculation
        self.last_ticks = current_ticks
        self.last_time = current_time
        
    def get_position_degrees(self):
        # Return current angular position in degrees
        return self.position
        
    def get_speed(self):
        # Return filtered angular velocity
        return self.speed
        
    def get_raw_speed(self):
        # Return the unfiltered instantaneous velocity
        return self.raw_speed