import smbus
import math
import time

class MPU6050:
    POWER_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43

    ACCEL_SCALE_FACTOR = 16384.0
    GYRO_SCALE_FACTOR = 131.0

    def __init__(self, address=0x68, bus_num=1):
        self.address = address
        self.bus = smbus.SMBus(bus_num)
        self.acc_x, self.acc_y, self.acc_z = 0.0, 0.0, 0.0
        self.gyro_x, self.gyro_y, self.gyro_z = 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.acc_error_x, self.acc_error_y = 0.0, 0.0
        self.gyro_error_x, self.gyro_error_y, self.gyro_error_z = 0.0, 0.0, 0.0
        self.elapsed_time = 0.01

        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, self.POWER_MGMT_1, 0)
        print("MPU6050 initialized")

    def calibrate(self):
        print("Calibrating IMU...")
        samples = 200
        for _ in range(samples):
            acc_data = self.read_raw_accel()
            gyro_data = self.read_raw_gyro()
            self.acc_error_x += math.atan2(acc_data[1], acc_data[2]) * (180 / math.pi)
            self.acc_error_y += math.atan2(-acc_data[0], math.sqrt(acc_data[1]**2 + acc_data[2]**2)) * (180 / math.pi)
            self.gyro_error_x += gyro_data[0] / self.GYRO_SCALE_FACTOR
            self.gyro_error_y += gyro_data[1] / self.GYRO_SCALE_FACTOR
            self.gyro_error_z += gyro_data[2] / self.GYRO_SCALE_FACTOR

        # Average errors
        self.acc_error_x /= samples
        self.acc_error_y /= samples
        self.gyro_error_x /= samples
        self.gyro_error_y /= samples
        self.gyro_error_z /= samples

    def read_raw_accel(self):
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 6)
        return [(data[i] << 8 | data[i+1]) - (65536 if data[i] > 127 else 0) for i in range(0, 6, 2)]

    def read_raw_gyro(self):
        data = self.bus.read_i2c_block_data(self.address, self.GYRO_XOUT_H, 6)
        return [(data[i] << 8 | data[i+1]) - (65536 if data[i] > 127 else 0) for i in range(0, 6, 2)]

    def update(self):
        acc_angles = {
            'roll': math.atan2(self.acc_y, self.acc_z) * (180 / math.pi) - self.acc_error_x,
            'pitch': math.atan2(-self.acc_x, math.sqrt(self.acc_y**2 + self.acc_z**2)) * (180 / math.pi) - self.acc_error_y,
        }
        
        gyro_rates = {
            'roll_rate': (self.gyro_x - self.gyro_error_x) * self.elapsed_time,
            'pitch_rate': (self.gyro_y - self.gyro_error_y) * self.elapsed_time  # Fixed variable name
        }  # Added closing brace

        # Complementary filter implementation
        self.roll = 0.96 * (self.roll + gyro_rates['roll_rate']) + 0.04 * acc_angles['roll']
        self.pitch = 0.96 * (self.pitch + gyro_rates['pitch_rate']) + 0.04 * acc_angles['pitch']

        return {
            'accel': {'x': self.acc_x, 'y': self.acc_y, 'z': self.acc_z},
            'gyro': {'x': self.gyro_x, 'y': self.gyro_y, 'z': self.gyro_z},
            'angle': {'roll': self.roll, 'pitch': self.pitch}
        }