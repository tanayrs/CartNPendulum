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
class TrivikramEncoderProcessor:
    def __init__(self, pulses_per_rev, play_degrees, is_wheel=True, 
                 is_front=True, cutoff_freq=1.0, sample_time=0.001):
        self.ticks_per_rev = 4 * pulses_per_rev
        self.play_degrees = play_degrees
        self.is_wheel = is_wheel
        self.is_front = is_front
        self.cutoff_freq = cutoff_freq
        self.sample_time = sample_time
        
        if self.is_wheel:
            self.deadband_upper = play_degrees / 2.0
            self.deadband_lower = -play_degrees / 2.0
        else:
            self.deadband_upper = (play_degrees / 2.0) * self.ticks_per_rev / 360.0
            self.deadband_lower = -self.deadband_upper
            
        self.adjusted_ticks = 0
        self.last_ticks = 0
        self.speed = 0.0
        self.bevel_ratio = 1.0
        self.steer_ppr = 2264

    def update(self, raw_ticks, steer_ticks=0, steer_offset=0):
        if self.is_wheel:
            self._update_wheel(raw_ticks, steer_ticks, steer_offset)
        else:
            self._update_steering(raw_ticks)
            
        self._calculate_speed(raw_ticks)

    def _update_wheel(self, ticks, steer_ticks, steer_offset):
        coupled_deg = self._compute_coupled_rotation(ticks, steer_ticks, steer_offset)
        if coupled_deg > self.deadband_upper:
            self.adjusted_ticks += (coupled_deg - self.deadband_upper)
            self.deadband_upper = coupled_deg
            self.deadband_lower = coupled_deg - self.play_degrees
        elif coupled_deg < self.deadband_lower:
            self.adjusted_ticks += (coupled_deg - self.deadband_lower)
            self.deadband_lower = coupled_deg
            self.deadband_upper = coupled_deg + self.play_degrees

    def _compute_coupled_rotation(self, wheel_ticks, steer_ticks, offset):
        wheel_deg = (wheel_ticks / self.ticks_per_rev) * 360
        steer_deg = 360 * (steer_ticks - offset) / (self.bevel_ratio * self.steer_ppr)
        return wheel_deg - steer_deg if self.is_front else wheel_deg + steer_deg

    def _calculate_speed(self, current_ticks):
        delta = current_ticks - self.last_ticks
        self.speed = (delta * 360 / self.ticks_per_rev) / self.sample_time
        self.last_ticks = current_ticks

    def get_position_degrees(self):
        return self.adjusted_ticks % 360

    def get_speed(self):
        return self.speed
    
# Initialize hardware encoder
encoder = PiEncoder(pin_a=23, pin_b=24)

# Initialize data processor
processor = TrivikramEncoderProcessor(
    pulses_per_rev=1000,
    play_degrees=5.0,
    is_wheel=True
)

while True:
    raw_ticks = encoder.read()
    processor.update(raw_ticks)
    print(f"Position: {processor.get_position_degrees():.2f}°")
    print(f"Speed: {processor.get_speed():.2f}°/s")
    time.sleep(0.01)