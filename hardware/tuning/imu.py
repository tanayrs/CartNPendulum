#!/usr/bin/env python3

import smbus
import math
import time
import csv
import os
from datetime import datetime

class MPU6050:
    """Class to handle MPU6050 sensor operations"""
    
    # MPU6050 Register Addresses
    POWER_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    
    # Scale Factors
    ACCEL_SCALE_FACTOR = 16384.0  # for ±2g range
    GYRO_SCALE_FACTOR = 131.0     # for ±250°/s range
    
    def __init__(self, roll_offset=0.0, address=0x68, bus_num=1):
        """Initialize the MPU6050 sensor"""
        self.address = address
        self.bus = smbus.SMBus(bus_num)
        
        # Initialize variables for angle calculation
        self.acc_x, self.acc_y, self.acc_z = 0.0, 0.0, 0.0
        self.gyro_x, self.gyro_y, self.gyro_z = 0.0, 0.0, 0.0
        
        self.acc_angle_x, self.acc_angle_y = 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        self.acc_error_x, self.acc_error_y = -2.77, -0.83 
        self.gyro_error_x, self.gyro_error_y, self.gyro_error_z = -3.90, -1.47, 0.20

        # Roll offset is angle measured when bot is "balanced"
        self.roll_offset = roll_offset
        
        self.elapsed_time, self.current_time, self.previous_time = 0.0, 0.0, 0.0
        self.current_time = time.time() * 1000  # Initialize current_time
        
        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, self.POWER_MGMT_1, 0)
        print("MPU6050 initialized")
    
    def calibrate(self, samples=200):
        """Calculate sensor offsets"""
        print("Calculating IMU offsets. Please keep the sensor still...")
        
        self.acc_error_x, self.acc_error_y = 0, 0
        self.gyro_error_x, self.gyro_error_y, self.gyro_error_z = 0, 0, 0
        
        # Calibrate accelerometer
        for _ in range(samples):
            try:
                acc_data = self.read_raw_accel()
                acc_x = acc_data[0] / self.ACCEL_SCALE_FACTOR
                acc_y = acc_data[1] / self.ACCEL_SCALE_FACTOR
                acc_z = acc_data[2] / self.ACCEL_SCALE_FACTOR
                
                # Calculate angles and sum up errors
                self.acc_error_x += (math.atan2(acc_y, acc_z) * 180 / math.pi)
                self.acc_error_y += (math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 180 / math.pi)
                
                time.sleep(0.003)
            except Exception as e:
                print(f"Error during accelerometer calibration: {e}")
        
        # Calculate average accelerometer error
        self.acc_error_x /= samples
        self.acc_error_y /= samples
        
        # Calibrate gyroscope
        for _ in range(samples):
            try:
                gyro_data = self.read_raw_gyro()
                gyro_x = gyro_data[0] / self.GYRO_SCALE_FACTOR
                gyro_y = gyro_data[1] / self.GYRO_SCALE_FACTOR
                gyro_z = gyro_data[2] / self.GYRO_SCALE_FACTOR
                
                # Sum up errors
                self.gyro_error_x += gyro_x
                self.gyro_error_y += gyro_y
                self.gyro_error_z += gyro_z
                
                time.sleep(0.003)
            except Exception as e:
                print(f"Error during gyroscope calibration: {e}")
        
        # Calculate average gyroscope error
        self.gyro_error_x /= samples
        self.gyro_error_y /= samples
        self.gyro_error_z /= samples
        
        print("IMU calibration complete.")
        print(f"AccErrorX: {self.acc_error_x:.2f}")
        print(f"AccErrorY: {self.acc_error_y:.2f}")
        print(f"GyroErrorX: {self.gyro_error_x:.2f}")
        print(f"GyroErrorY: {self.gyro_error_y:.2f}")
        print(f"GyroErrorZ: {self.gyro_error_z:.2f}")
    
    def read_raw_accel(self):
        try:
            data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 6)
        except OSError as e:
            print(f"Error reading accelerometer: {e}")
            if hasattr(self, 'raw_accel_data'):
                return self.raw_accel_data  # Return last successful reading
            return [0]*6  # Fallback for initial failure
    
        # Store successful reading for future error fallback
        self.raw_accel_data = data
    
        # Original processing continues
        raw_x = (data[0] << 8) | data[1]
        raw_y = (data[2] << 8) | data[3]
        raw_z = (data[4] << 8) | data[5]
        return [raw_x, raw_y, raw_z]

    def read_raw_gyro(self):
        """Read raw gyroscope data from the sensor"""
        data = self.bus.read_i2c_block_data(self.address, self.GYRO_XOUT_H, 6)
        x = (data[0] << 8) | data[1]
        y = (data[2] << 8) | data[3]
        z = (data[4] << 8) | data[5]
        
        # Convert from two's complement
        if x > 0x7FFF:
            x -= 0x10000
        if y > 0x7FFF:
            y -= 0x10000
        if z > 0x7FFF:
            z -= 0x10000
            
        return [x, y, z]
    
    def get_accel_data(self):
        """Get processed accelerometer data in g units"""
        raw_data = self.read_raw_accel()
        self.acc_x = raw_data[0] / self.ACCEL_SCALE_FACTOR
        self.acc_y = raw_data[1] / self.ACCEL_SCALE_FACTOR
        self.acc_z = raw_data[2] / self.ACCEL_SCALE_FACTOR
        
        return {'x': self.acc_x, 'y': self.acc_y, 'z': self.acc_z}
    
    def get_gyro_data(self):
        """Get processed gyroscope data in degrees/s"""
        raw_data = self.read_raw_gyro()
        self.gyro_x = raw_data[0] / self.GYRO_SCALE_FACTOR - self.gyro_error_x
        self.gyro_y = raw_data[1] / self.GYRO_SCALE_FACTOR - self.gyro_error_y
        self.gyro_z = raw_data[2] / self.GYRO_SCALE_FACTOR - self.gyro_error_z
        
        return {'x': self.gyro_x, 'y': self.gyro_y, 'z': self.gyro_z}
    
    def read_temp(self):
        """Read temperature from the sensor in degrees Celsius"""
        data = self.bus.read_i2c_block_data(self.address, 0x41, 2)
        temp = (data[0] << 8) | data[1]
        
        # Convert from two's complement
        if temp > 0x7FFF:
            temp -= 0x10000
            
        # Formula from datasheet
        return (temp / 340.0) + 36.53
    
    def get_angles_accelerometer(self):
        """Calculate angles from accelerometer data only"""
        # Get accelerometer data
        self.get_accel_data()
        
        # Calculate angles using atan2 for better accuracy
        roll = math.atan2(self.acc_y, self.acc_z) * 180 / math.pi
        pitch = math.atan2(-self.acc_x, math.sqrt(self.acc_y**2 + self.acc_z**2)) * 180 / math.pi
        
        return {'roll': roll - self.acc_error_x, 'pitch': pitch - self.acc_error_y}
    
    def update(self):
        """Update all sensor values and calculate angles using complementary filter"""
        # Get accelerometer and gyroscope data
        self.get_accel_data()
        self.get_gyro_data()
        
        # Calculate angles from accelerometer
        acc_angles = self.get_angles_accelerometer()
        self.acc_angle_x = acc_angles['roll']
        self.acc_angle_y = acc_angles['pitch']
        
        # Calculate elapsed time for integration
        self.previous_time = self.current_time
        self.current_time = time.time() * 1000  # milliseconds
        self.elapsed_time = (self.current_time - self.previous_time) / 1000  # seconds
        
        # Prevent large time intervals that could cause jumps in angle calculation
        if self.elapsed_time > 0.1:
            self.elapsed_time = 0.1
        
        # Apply complementary filter
        # The filter combines accelerometer angles (which are absolute but noisy)
        # with integrated gyroscope data (which is smooth but drifts over time)
        self.roll = 0.96 * (self.roll + self.gyro_x * self.elapsed_time) + 0.04 * self.acc_angle_x
        self.pitch = 0.96 * (self.pitch + self.gyro_y * self.elapsed_time) + 0.04 * self.acc_angle_y
        
        # For yaw, we can only use gyroscope as accelerometer cannot provide yaw information
        self.yaw += self.gyro_z * self.elapsed_time

        # Optionally reset yaw to 0 if needed
        # self.yaw = 0  # Uncomment if you want to reset yaw
        
        return {
            'accel': {'x': self.acc_x, 'y': self.acc_y, 'z': self.acc_z},
            'gyro': {'x': self.gyro_x, 'y': self.gyro_y, 'z': self.gyro_z},
            'angle': {'roll': self.roll - self.roll_offset, 'pitch': self.pitch, 'yaw': self.yaw},
            'temp': self.read_temp()
        }
