#!/usr/bin/env python3

# File used to find calibration values of theta and theta_dot sensor

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
    
    def __init__(self, address=0x68, bus_num=1):
        """Initialize the MPU6050 sensor"""
        self.address = address
        self.bus = smbus.SMBus(bus_num)
        
        # Initialize variables for angle calculation
        self.acc_x, self.acc_y, self.acc_z = 0.0, 0.0, 0.0
        self.gyro_x, self.gyro_y, self.gyro_z = 0.0, 0.0, 0.0
        
        self.acc_angle_x, self.acc_angle_y = 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        
        self.acc_error_x, self.acc_error_y = 0.0, 0.0
        self.gyro_error_x, self.gyro_error_y, self.gyro_error_z = 0.0, 0.0, 0.0
        
        self.elapsed_time, self.current_time, self.previous_time = 0.0, 0.0, 0.0
        self.current_time = time.time() * 1000  # Initialize current_time
        
        # Wake up the MPU6050
        self.bus.write_byte_data(self.address, self.POWER_MGMT_1, 0)
        print("MPU6050 initialized")
    
    def calibrate(self, samples=200):
        """Calculate sensor offsets"""
        print("Calculating IMU offsets. Please keep the sensor still...")
        
        # Initialize error variables to zero
        self.acc_error_x, self.acc_error_y = 0, 0
        self.gyro_error_x, self.gyro_error_y, self.gyro_error_z = 0, 0, 0
        
        # Calibrate accelerometer
        for _ in range(samples):
            try:
                # Read raw accelerometer data
                acc_data = self.read_raw_accel()
                # Convert raw values to g units
                acc_x = acc_data[0] / self.ACCEL_SCALE_FACTOR
                acc_y = acc_data[1] / self.ACCEL_SCALE_FACTOR
                acc_z = acc_data[2] / self.ACCEL_SCALE_FACTOR
                
                # Calculate roll angle (x-axis rotation) using arctan of y/z components
                self.acc_error_x += (math.atan2(acc_y, acc_z) * 180 / math.pi)
                # Calculate pitch angle (y-axis rotation) using arctan of x and sqrt(y²+z²)
                self.acc_error_y += (math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 180 / math.pi)
                
                # Short delay between readings
                time.sleep(0.003)
            except Exception as e:
                print(f"Error during accelerometer calibration: {e}")
        
        # Calculate average accelerometer error over all samples
        self.acc_error_x /= samples
        self.acc_error_y /= samples
        
        # Calibrate gyroscope
        for _ in range(samples):
            try:
                # Read raw gyroscope data
                gyro_data = self.read_raw_gyro()
                # Convert raw values to degrees/second
                gyro_x = gyro_data[0] / self.GYRO_SCALE_FACTOR
                gyro_y = gyro_data[1] / self.GYRO_SCALE_FACTOR
                gyro_z = gyro_data[2] / self.GYRO_SCALE_FACTOR
                
                # Sum up gyroscope readings (ideally should be zero when still)
                self.gyro_error_x += gyro_x
                self.gyro_error_y += gyro_y
                self.gyro_error_z += gyro_z
                
                # Short delay between readings
                time.sleep(0.003)
            except Exception as e:
                print(f"Error during gyroscope calibration: {e}")
        
        # Calculate average gyroscope error over all samples
        self.gyro_error_x /= samples
        self.gyro_error_y /= samples
        self.gyro_error_z /= samples
        
        # Output calibration results
        print("IMU calibration complete.")
        print(f"AccErrorX: {self.acc_error_x:.2f}")  # Roll error in degrees
        print(f"AccErrorY: {self.acc_error_y:.2f}")  # Pitch error in degrees
        print(f"GyroErrorX: {self.gyro_error_x:.2f}")  # X-axis drift in degrees/second
        print(f"GyroErrorY: {self.gyro_error_y:.2f}")  # Y-axis drift in degrees/second
        print(f"GyroErrorZ: {self.gyro_error_z:.2f}")  # Z-axis drift in degrees/second
    
    def read_raw_accel(self):
        """Read raw accelerometer data from the sensor"""
        # Read 6 bytes starting from ACCEL_XOUT_H register
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 6)
        # Combine high and low bytes to form 16-bit values
        x = (data[0] << 8) | data[1]  # X-axis value
        y = (data[2] << 8) | data[3]  # Y-axis value
        z = (data[4] << 8) | data[5]  # Z-axis value
        
        # Convert from two's complement for negative values
        if x > 0x7FFF:
            x -= 0x10000  # If highest bit is set, value is negative
        if y > 0x7FFF:
            y -= 0x10000  # If highest bit is set, value is negative
        if z > 0x7FFF:
            z -= 0x10000  # If highest bit is set, value is negative
            
        return [x, y, z]  # Return raw accelerometer values
    
    def read_raw_gyro(self):
        """Read raw gyroscope data from the sensor"""
        # Read 6 bytes starting from GYRO_XOUT_H register
        data = self.bus.read_i2c_block_data(self.address, self.GYRO_XOUT_H, 6)
        # Combine high and low bytes to form 16-bit values
        x = (data[0] << 8) | data[1]  # X-axis value
        y = (data[2] << 8) | data[3]  # Y-axis value
        z = (data[4] << 8) | data[5]  # Z-axis value
        
        # Convert from two's complement for negative values
        if x > 0x7FFF:
            x -= 0x10000  # If highest bit is set, value is negative
        if y > 0x7FFF:
            y -= 0x10000  # If highest bit is set, value is negative
        if z > 0x7FFF:
            z -= 0x10000  # If highest bit is set, value is negative
            
        return [x, y, z]  # Return raw gyroscope values
    
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
        # Read 2 bytes from the temperature sensor register (0x41)
        data = self.bus.read_i2c_block_data(self.address, 0x41, 2)
        # Combine high byte (shifted left by 8 bits) with low byte to form 16-bit value
        temp = (data[0] << 8) | data[1]
        
        # Convert from two's complement for negative values
        if temp > 0x7FFF:
            temp -= 0x10000
            
        # Apply scaling formula from MPU6050 datasheet
        # Temperature in degrees C = (raw_value / 340) + 36.53
        return (temp / 340.0) + 36.53
    
    def get_angles_accelerometer(self):
        """Calculate angles from accelerometer data only"""
        # Get accelerometer data
        self.get_accel_data()
        
        # Calculate angles using atan2 for better accuracy
        # roll (rotation around X-axis) is calculated from Y and Z acceleration components
        roll = math.atan2(self.acc_y, self.acc_z) * 180 / math.pi
        # pitch (rotation around Y-axis) is calculated using X and combined Y-Z acceleration
        pitch = math.atan2(-self.acc_x, math.sqrt(self.acc_y**2 + self.acc_z**2)) * 180 / math.pi
        
        # Return angles with calibration error compensation
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
            'angle': {'roll': self.roll, 'pitch': self.pitch, 'yaw': self.yaw},
            'temp': self.read_temp()
        }


class DataLogger:
    """Class to handle data logging"""
    
    def __init__(self, log_dir="imu_data"):
        self.log_dir = log_dir
        self.log_filename = ""
        self.log_file = None
        self.log_writer = None
        
        # Create logs directory if it doesn't exist
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
    
    def init_logging(self):
        """Initialize the CSV logging"""
        # Create a new log file with timestamp in the name
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = f"{self.log_dir}/imu_log_{timestamp}.csv"
        
        # Open the log file and write header
        self.log_file = open(self.log_filename, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        
        # Write header row
        self.log_writer.writerow([
            'timestamp_ms',
            'AccX', 'AccY', 'AccZ',
            'GyroX', 'GyroY', 'GyroZ',
            'roll', 'pitch', 'yaw',
            'temp'
        ])
        
        print(f"Logging data to {self.log_filename}")
        
        return self.log_writer
    
    def log_data(self, data):
        """Log the IMU data to the CSV file"""
        if self.log_writer is None:
            print("Log writer not initialized. Call init_logging() first.")
            return
        
        # Get timestamp in milliseconds
        timestamp_ms = int(time.time() * 1000)
        
        # Extract data from the sensor readings dictionary
        accel = data['accel']   # Accelerometer data (x, y, z)
        gyro = data['gyro']     # Gyroscope data (x, y, z)
        angle = data['angle']   # Calculated angles (roll, pitch, yaw)
        temp = data['temp']     # Temperature reading in Celsius
        
        # Write data row to CSV with 4 decimal precision for sensor values
        self.log_writer.writerow([
            timestamp_ms,                     # Current time in milliseconds
            round(accel['x'], 4),             # Acceleration X (g)
            round(accel['y'], 4),             # Acceleration Y (g)
            round(accel['z'], 4),             # Acceleration Z (g)
            round(gyro['x'], 4),              # Angular velocity X (°/s)
            round(gyro['y'], 4),              # Angular velocity Y (°/s)
            round(gyro['z'], 4),              # Angular velocity Z (°/s)
            round(angle['roll'], 4),          # Roll angle (°)
            round(angle['pitch'], 4),         # Pitch angle (°)
            round(angle['yaw'], 4),           # Yaw angle (°)
            round(temp, 2)                    # Temperature (°C)
        ])
        
        # Flush to ensure data is written to disk immediately
        self.log_file.flush()
    
    def close(self):
        """Close the log file"""
        if self.log_file is not None:
            self.log_file.close()
            print(f"Data logged to {self.log_filename}")


def print_imu_data(data):
    """Print the IMU data to the terminal"""
    accel = data['accel']
    gyro = data['gyro']
    angle = data['angle']
    temp = data['temp']
    
    print(f"Accelerometer: X={accel['x']:.2f}g, Y={accel['y']:.2f}g, Z={accel['z']:.2f}g")
    print(f"Gyroscope: X={gyro['x']:.2f}°/s, Y={gyro['y']:.2f}°/s, Z={gyro['z']:.2f}°/s")
    print(f"Angle: Roll={angle['roll']:.2f}°, Pitch={angle['pitch']:.2f}°, Yaw={angle['yaw']:.2f}°")
    print(f"Temperature: {temp:.2f}°C")
    print("-" * 50)


# Main function
def main():
    print("MPU6050 IMU Reader (Fixed Angle Calculation)")
    print("--------------------------------------------")
    
    try:
        # Initialize the MPU6050
        mpu = MPU6050()
        
        # Calibrate the sensor
        mpu.calibrate()
        
        # Initialize the data logger
        logger = DataLogger()
        
        # Ask if user wants to log data
        log_choice = input("Do you want to log data to CSV? (y/n): ").lower()
        if log_choice == 'y':
            logger.init_logging()
            logging_enabled = True
        else:
            logging_enabled = False
        
        print("\nReading IMU data... Press CTRL+C to stop.\n")
        
        # Initialize timing variables
        sample_rate = 10  # Hz (samples per second)
        sample_interval = 1.0 / sample_rate
        last_sample_time = time.time()
        
        # Main loop to continuously read and display sensor data
        while True:
            current_time = time.time()
            
            # Check if it's time for the next sample
            if current_time - last_sample_time >= sample_interval:
                # Update sensor data
                data = mpu.update()
                
                # Display data
                print_imu_data(data)
                
                # Log data if enabled
                if logging_enabled:
                    logger.log_data(data)
                
                last_sample_time = current_time
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
    
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'logging_enabled' in locals() and logging_enabled and 'logger' in locals():
            logger.close()


if __name__ == "__main__":
    main()