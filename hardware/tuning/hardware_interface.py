# File 2: hardware_interface.py
import time
import numpy as np
from imu import MPU6050
from motor import HardwarePWMMotor
from encoder import PiEncoder, EncoderProcessor

class HardwareInterface:
    def __init__(self):
        self.motor = HardwarePWMMotor()
        self.imu = MPU6050()
        self.encoder = PiEncoder(pin_a=23, pin_b=24)
        self.processor = EncoderProcessor(pulses_per_rev=2262)
        self.emergency_stop = False
        
    def get_observation(self):
        imu_data = self.imu.update()
        encoder_ticks = self.encoder.read()
        self.processor.update(encoder_ticks)
        
        return np.array([
            self.processor.get_position_meters(),
            self.processor.get_speed_ms(),
            np.radians(imu_data['angle']['roll']),
            imu_data['gyro']['x']
        ], dtype=np.float32)

    def execute_action(self, normalized_action):
        pwm = np.clip(normalized_action * 40000, -40000, 40000)
        current_speed = self.processor.get_speed_ms()
        self.motor.set_speed(pwm, current_speed)
        
        # Safety checks
        if abs(self.processor.get_position_meters()) > 0.5:
            self.emergency_stop = True
        if abs(self.imu.update()['angle']['roll']) > 25:
            self.emergency_stop = True

    def safe_reset(self):
        self.motor.stop()
        self.emergency_stop = False
        input("Reset robot to starting position and press Enter...")
        time.sleep(1)  # Allow sensors to stabilize

