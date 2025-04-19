import pigpio
import time
import math

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

# TrivikramEncoderProcessor remains unchanged from previous implementation
class EncoderProcessor:
    def __init__(self, pulses_per_rev,cutoff_freq=1.0):
        self.ticks_per_rev = 4 * pulses_per_rev
        self.cutoff_freq = cutoff_freq
        
        self.last_ticks = 0
        self.position = 0
        self.speed = 0.0

        self.last_time = None

    def update(self, raw_ticks):
        self._update_wheel(raw_ticks)
            
        self._calculate_speed(raw_ticks)

    def _update_wheel(self, ticks):
        self.position = (ticks / self.ticks_per_rev) * 360

    def _calculate_speed(self, current_ticks):
        current_time = time.perf_counter()

        if self.last_time is None:
            self.last_time = current_time
            self.last_ticks = current_ticks
            self.speed = 0.0
            return

        delta_ticks = current_ticks - self.last_ticks
        delta_time = current_time - self.last_time

        if delta_time > 0:
            self.speed = (delta_ticks * 360 / self.ticks_per_rev) / delta_time

        self.last_ticks = current_ticks
        self.last_time = current_time

    def get_position_degrees(self):
        return self.position

    def get_speed(self):
        return self.speed
