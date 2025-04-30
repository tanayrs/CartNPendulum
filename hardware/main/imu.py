# Class to measure theta and theta_dot using MPU6050

'''
MPU6050 IMU Sensor Interface Module

This module provides a comprehensive interface for the MPU6050 Inertial Measurement Unit (IMU) used in the Cart-Pendulum reinforcement learning platform. It handles sensor communication, data acquisition, and sophisticated signal processing to obtain accurate orientation data.

Features:
- I2C communication with MPU6050 accelerometer and gyroscope
- Sensor calibration to account for manufacturing offsets
- Complementary filtering for stable angle estimation
- Accelerometer and gyroscope data fusion
- Roll, pitch, and yaw angle calculation
- Temperature reading capabilities
- Error handling for robust operation

Measurements:
- Accelerometer data (x, y, z) in g units
- Gyroscope data (x, y, z) in degrees/second
- Roll angle: Used as pendulum theta in the cart-pendulum system
- Angular velocity: Used as theta_dot in control algorithms
- Temperature: For monitoring sensor operating conditions

Usage:
    imu = MPU6050(roll_offset=0.0)  # Initialize with optional offset
    imu.calibrate()  # Calibrate to remove sensor bias
    
    # In main control loop
    data = imu.update()
    theta = data['angle']['roll']  # Get pendulum angle
    theta_dot = data['gyro']['x']  # Get angular velocity
    
    # Access other data if needed
    accel_data = data['accel']  # Raw acceleration values
    temp = data['temp']  # Temperature in Celsius

Dependencies:
    - smbus: For I2C communication
    - math: For trigonometric calculations
    - time: For timing operations
'''

#!/usr/bin/env python3

# imports
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
        
        # Initialize error variables
        self.acc_error_x, self.acc_error_y = 0, 0
        self.gyro_error_x, self.gyro_error_y, self.gyro_error_z = 0, 0, 0
        
        # Calibrate accelerometer by collecting multiple samples
        for _ in range(samples):
            try:
                acc_data = self.read_raw_accel()  # Get raw accelerometer readings
                acc_x = acc_data[0] / self.ACCEL_SCALE_FACTOR  # Convert to g units
                acc_y = acc_data[1] / self.ACCEL_SCALE_FACTOR
                acc_z = acc_data[2] / self.ACCEL_SCALE_FACTOR
                
                # Calculate angle errors from accelerometer data
                self.acc_error_x += (math.atan2(acc_y, acc_z) * 180 / math.pi)  # Roll angle in degrees
                self.acc_error_y += (math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 180 / math.pi)  # Pitch angle in degrees
                
                time.sleep(0.003)  # Short delay between readings
            except Exception as e:
                print(f"Error during accelerometer calibration: {e}")
        
        # Calculate average accelerometer error
        self.acc_error_x /= samples
        self.acc_error_y /= samples
        
        # Calibrate gyroscope by collecting multiple samples
        for _ in range(samples):
            try:
                gyro_data = self.read_raw_gyro()  # Get raw gyroscope readings
                gyro_x = gyro_data[0] / self.GYRO_SCALE_FACTOR  # Convert to degrees/second
                gyro_y = gyro_data[1] / self.GYRO_SCALE_FACTOR
                gyro_z = gyro_data[2] / self.GYRO_SCALE_FACTOR
                
                # Sum up gyroscope readings (should be zero when stationary)
                self.gyro_error_x += gyro_x
                self.gyro_error_y += gyro_y
                self.gyro_error_z += gyro_z
                
                time.sleep(0.003)  # Short delay between readings
            except Exception as e:
                print(f"Error during gyroscope calibration: {e}")
        
        # Calculate average gyroscope error
        self.gyro_error_x /= samples
        self.gyro_error_y /= samples
        self.gyro_error_z /= samples
        
        # Print calibration results
        print("IMU calibration complete.")
        print(f"AccErrorX: {self.acc_error_x:.2f}")
        print(f"AccErrorY: {self.acc_error_y:.2f}")
        print(f"GyroErrorX: {self.gyro_error_x:.2f}")
        print(f"GyroErrorY: {self.gyro_error_y:.2f}")
        print(f"GyroErrorZ: {self.gyro_error_z:.2f}")
    
    def read_raw_accel(self):
        """Read raw accelerometer data from the sensor"""
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 6)  # Read 6 bytes starting from ACCEL_XOUT_H
        x = (data[0] << 8) | data[1]  # Combine high and low bytes for X axis
        y = (data[2] << 8) | data[3]  # Combine high and low bytes for Y axis
        z = (data[4] << 8) | data[5]  # Combine high and low bytes for Z axis
        
        # Convert from two's complement (16-bit signed values)
        if x > 0x7FFF:
            x -= 0x10000
        if y > 0x7FFF:
            y -= 0x10000
        if z > 0x7FFF:
            z -= 0x10000
            
        return [x, y, z]
    
    def read_raw_gyro(self):
        """Read raw gyroscope data from the sensor"""
        data = self.bus.read_i2c_block_data(self.address, self.GYRO_XOUT_H, 6)  # Read 6 bytes starting from GYRO_XOUT_H
        x = (data[0] << 8) | data[1]  # Combine high and low bytes for X axis
        y = (data[2] << 8) | data[3]  # Combine high and low bytes for Y axis
        z = (data[4] << 8) | data[5]  # Combine high and low bytes for Z axis
        
        # Convert from two's complement (16-bit signed values)
        if x > 0x7FFF:
            x -= 0x10000
        if y > 0x7FFF:
            y -= 0x10000
        if z > 0x7FFF:
            z -= 0x10000
            
        return [x, y, z]
    
    def get_accel_data(self):
        """Get processed accelerometer data in g units"""
        raw_data = self.read_raw_accel()  # Get raw accelerometer readings
        self.acc_x = raw_data[0] / self.ACCEL_SCALE_FACTOR  # Convert to g units (±2g range)
        self.acc_y = raw_data[1] / self.ACCEL_SCALE_FACTOR
        self.acc_z = raw_data[2] / self.ACCEL_SCALE_FACTOR
        
        return {'x': self.acc_x, 'y': self.acc_y, 'z': self.acc_z}
    
    def get_gyro_data(self):
        """Get processed gyroscope data in degrees/s"""
        raw_data = self.read_raw_gyro()  # Get raw gyroscope readings
        self.gyro_x = raw_data[0] / self.GYRO_SCALE_FACTOR - self.gyro_error_x  # Convert to degrees/s and apply calibration
        self.gyro_y = raw_data[1] / self.GYRO_SCALE_FACTOR - self.gyro_error_y
        self.gyro_z = raw_data[2] / self.GYRO_SCALE_FACTOR - self.gyro_error_z
        
        return {'x': self.gyro_x, 'y': self.gyro_y, 'z': self.gyro_z}
    
    def read_temp(self):
        """Read temperature from the sensor in degrees Celsius"""
        data = self.bus.read_i2c_block_data(self.address, 0x41, 2)  # Read 2 bytes from temperature register
        temp = (data[0] << 8) | data[1]  # Combine high and low bytes
        
        # Convert from two's complement
        if temp > 0x7FFF:
            temp -= 0x10000
            
        # Formula from MPU6050 datasheet to convert to degrees Celsius
        return (temp / 340.0) + 36.53
    
    def get_angles_accelerometer(self):
        """Calculate angles from accelerometer data only"""
        # Get accelerometer data
        self.get_accel_data()
        
        # Calculate roll and pitch angles using arctan (in degrees)
        roll = math.atan2(self.acc_y, self.acc_z) * 180 / math.pi  # Roll around X axis
        pitch = math.atan2(-self.acc_x, math.sqrt(self.acc_y**2 + self.acc_z**2)) * 180 / math.pi  # Pitch around Y axis
        
        # Apply calibration offsets
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
