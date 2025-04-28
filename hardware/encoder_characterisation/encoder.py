# needed to read from encoder

import pigpio
import time
import math
import numpy as np
from collections import deque

class PiEncoder:
    def __init__(self, pin_a, pin_b):
        self.pi = pigpio.pi()
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        self.state = 0
        
        # Set pull-ups
        self.pi.set_pull_up_down(pin_a, pigpio.PUD_UP)
        self.pi.set_pull_up_down(pin_b, pigpio.PUD_UP)
        
        # Get initial state
        self.last_a = self.pi.read(pin_a)
        self.last_b = self.pi.read(pin_b)
        
        # Set up callbacks with pigpio
        self.cb_a = self.pi.callback(pin_a, pigpio.EITHER_EDGE, self._update)
        self.cb_b = self.pi.callback(pin_b, pigpio.EITHER_EDGE, self._update)
        
    def _update(self, gpio, level, tick):
        a = self.pi.read(self.pin_a)
        b = self.pi.read(self.pin_b)
        state = (self.last_a << 1) | self.last_b
        state |= (a << 3) | (b << 2)
        
        transition_table = {
            0b0000: 0, 0b0001: 1, 0b0010: -1, 0b0011: 2,
            0b0100: -1, 0b0101: 0, 0b0110: -2, 0b0111: 1,
            0b1000: 1, 0b1001: -2, 0b1010: 0, 0b1011: -1,
            0b1100: 2, 0b1101: -1, 0b1110: 1, 0b1111: 0
        }
        
        self.position += transition_table.get(state, 0)
        self.last_a = a
        self.last_b = b
        
    def read(self):
        return self.position
        
    def read_and_reset(self):
        pos = self.position
        self.position = 0
        return pos
        
    def cleanup(self):
        self.cb_a.cancel()
        self.cb_b.cancel()
        self.pi.stop()

class EncoderProcessor:
    def __init__(self, pulses_per_rev, window_size=8, wheel_rad = 0.1, cutoff_freq=1.0):
        self.ticks_per_rev = 4 * pulses_per_rev
        self.cutoff_freq = cutoff_freq
        self.window_size = window_size
        
        self.last_ticks = 0
        self.position = 0
        self.raw_speed = 0.0
        self.wheel_rad = wheel_rad
        self.speed = 0.0
        self.last_time = None
        
        # Create a deque to store the last window_size velocity measurements
        self.velocity_buffer = deque(maxlen=window_size)
        
    def update(self, raw_ticks):
        self._update_wheel(raw_ticks)
        self._calculate_speed(raw_ticks)
        
    def _update_wheel(self, ticks):
        self.position = (ticks / self.ticks_per_rev) * 2 * np.pi 
        
    def _calculate_speed(self, current_ticks):
        current_time = time.perf_counter()
        if self.last_time is None:
            self.last_time = current_time
            self.last_ticks = current_ticks
            self.raw_speed = 0.0
            # Initialize buffer with zeros
            for _ in range(self.window_size):
                self.velocity_buffer.append(0.0)
            return
            
        delta_ticks = current_ticks - self.last_ticks
        delta_time = current_time - self.last_time
        
        if delta_time > 0:
            # Calculate raw velocity
            raw_velocity = (delta_ticks * 2 * np.pi/ self.ticks_per_rev) / delta_time
            self.raw_speed = raw_velocity
            
            # Add to buffer
            self.velocity_buffer.append(raw_velocity)
            
            # Calculate filtered speed (average of buffer)
            self.speed = sum(self.velocity_buffer) / len(self.velocity_buffer)
        
        self.last_ticks = current_ticks
        self.last_time = current_time
        
    def get_position_rad(self):
        return self.position

    def get_position_meters(self):
        return self.position*self.wheel_rad
        
    def get_speed(self):
        return self.speed

    def get_speed_ms(self):
        return self.speed*self.wheel_rad
        
    def get_raw_speed(self):
        """Returns the unfiltered speed value"""
        return self.raw_speed
