#!/usr/bin/env python3

# Try to import dependencies, install if missing
try:
    import smbus
except ImportError:
    print("Installing smbus library...")
    import subprocess
    subprocess.check_call(["sudo", "apt-get", "install", "-y", "python3-smbus"])
    import smbus

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("Installing RPi.GPIO library...")
    import subprocess
    subprocess.check_call(["sudo", "apt-get", "install", "-y", "python3-rpi.gpio"])
    import RPi.GPIO as GPIO

import math
import time
import csv
import os
from datetime import datetime

# For hardware PWM
try:
    import pigpio
except ImportError:
    print("Installing pigpio library...")
    import subprocess
    subprocess.check_call(["sudo", "apt-get", "install", "-y", "pigpio", "python3-pigpio"])
    import pigpio

# Loop Time Definition
loopTimeConstant = 10000  # In micros
loopTimeConstSec = loopTimeConstant * 1e-6
sampling_time = loopTimeConstSec

# Encoder Parameters
wheelMotorPPR = 2264 // 4
pi = 3.141592
wheelRadius = 0.0525

# Motor and Encoder GPIO pins for Raspberry Pi
# For encoder: we'll use hardware-capable interrupt pins
encoderPin1 = 23  # GPIO 23
encoderPin2 = 24  # GPIO 24

# For motor control: using hardware PWM-capable pins
# Hardware PWM is available on GPIO 12, 13, 18, 19
pwm_pin = 18  # GPIO 18 for hardware PWM
dir_pin = 16  # GPIO 16 for direction

vel_cutoff_freq = 1000

# Position and velocity
position = 0.0
velocity = 0.0
last_position = 0.0
encoder_value = 0
last_encoded = 0

# Hardware PWM settings
PWM_FREQUENCY = 20000  # 20kHz frequency for smoother motor operation
PWM_RANGE = 40000    # Maximum resolution (1M steps)

# Deadband for Motor Inputs
deadzone = 0.0

# PID Setup
# Offsets For Balance Position
target_roll = 0.0
target_enc = 0.0
roll_offset = 3.7
int_pos = 0.0

# Offsets for Position Controller
target_position = 0.1
last_position_error = 0.0

# Error Initialization
error_roll = 0.0
last_error_roll = 0.0
error_enc = 0.0
last_error_enc = 0.0

# PID Setup
# Gains for Roll
Kp = 25.0  # theta gain
Kd = 0.0   # theta dot gain
Ki = 0.0

# Gains for Encoder
Kd_wheel = 0.0  # x_dot gain

# Gains for Position
Kp_pos = 1.0
Kd_pos = 0.0
Ki_pos = 0.0

# Error Function Initialization
u = 0.0
int_lean = 0.0

# PWM Value Initialization - now using the full range
pwm = 0.0
MAX_PWM = PWM_RANGE

# Sign of Roll and Previous Roll for Integral Wind-Up
sgnRoll, sgnPrevRoll = 0, 0
prev_time = 0
deadband_sign, prev_input_sign = 1, 1

# Data logging setup
log_dir = "robot_logs"
log_filename = ""
log_file = None
log_writer = None
log_interval = 0.1  # Log every 100ms
last_log_time = 0

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
        """Read raw accelerometer data from the sensor"""
        data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 6)
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
            'angle': {'roll': self.roll, 'pitch': self.pitch, 'yaw': self.yaw},
            'temp': self.read_temp()
        }
    def update(self, channel):
        # Use the channel parameter to read correct inputs
        MSB = GPIO.input(self.pin_a)
        LSB = GPIO.input(self.pin_b)
        
        encoded = (MSB << 1) | LSB
        sum_value = (self.last_encoded << 2) | encoded
        
        if sum_value == 0b1101 or sum_value == 0b0100 or sum_value == 0b0010 or sum_value == 0b1011:
            self.value += 1
        elif sum_value == 0b1110 or sum_value == 0b0111 or sum_value == 0b0001 or sum_value == 0b1000:
            self.value -= 1
            
        self.last_encoded = encoded
        GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=self.update)
    
    def update(self, channel):
        MSB = GPIO.input(self.pin_a)
        LSB = GPIO.input(self.pin_b)
        
        encoded = (MSB << 1) | LSB
        sum_value = (self.last_encoded << 2) | encoded
        
        if sum_value == 0b1101 or sum_value == 0b0100 or sum_value == 0b0010 or sum_value == 0b1011:
            self.value += 1
        elif sum_value == 0b1110 or sum_value == 0b0111 or sum_value == 0b0001 or sum_value == 0b1000:
            self.value -= 1
            
        self.last_encoded = encoded
    
    def read(self):
        return self.value

# Class for motor control using hardware PWM
class HardwarePWMMotor:
    def __init__(self, pwm_pin, dir_pin, pwm_range=PWM_RANGE, pwm_frequency=PWM_FREQUENCY):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.pwm_range = pwm_range
        
        # Setup direction pin
        GPIO.setup(dir_pin, GPIO.OUT)
        
        # Initialize pigpio for hardware PWM
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("Failed to connect to pigpio daemon. Starting daemon...")
            import subprocess
            subprocess.check_call(["sudo", "pigpiod"])
            time.sleep(1)
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise IOError("Could not connect to pigpio daemon")
        
        # Set PWM frequency
        self.pi.set_PWM_frequency(pwm_pin, pwm_frequency)
        
        # Set PWM range (resolution)
        self.pi.set_PWM_range(pwm_pin, pwm_range)
        
        # Start with 0 duty cycle
        self.pi.set_PWM_dutycycle(pwm_pin, 0)
    
    def set_speed(self, speed):
        # Speed should be between -MAX_PWM and MAX_PWM
        if speed < 0:
            GPIO.output(self.dir_pin, GPIO.LOW)  # Set direction backward
            speed = -speed
        else:
            GPIO.output(self.dir_pin, GPIO.HIGH)  # Set direction forward
        
        # Ensure speed is within range
        duty_cycle = min(abs(int(speed)), self.pwm_range)
        self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)
    
    def cleanup(self):
        self.pi.set_PWM_dutycycle(self.pwm_pin, 0)
        self.pi.stop()
    def update(self, ticks, steer_accumulated_ticks=0, steer_ticks_offset=0):
        # Parameters steer_accumulated_ticks and steer_ticks_offset are kept for compatibility
        self.current_ticks = ticks * (-1 if self.invert_direction else 1)
        
        # Calculate raw speed (ticks per second)
        raw_speed = (self.current_ticks - self.last_ticks) / self.sampling_time
        
        # Apply low-pass filter to speed
        self.filtered_speed = self.alpha * self.filtered_speed + (1 - self.alpha) * raw_speed
        
        self.last_ticks = self.current_ticks
        
        self.current_ticks = 0
        self.last_ticks = 0
        self.filtered_speed = 0.0
        self.alpha = 1.0 / (1.0 + vel_cutoff_freq * 2 * pi * sampling_time)
    
    def update(self, ticks, steer_accumulated_ticks, steer_ticks_offset):
        self.current_ticks = ticks * (-1 if self.invert_direction else 1)
        
        # Calculate raw speed (ticks per second)
        raw_speed = (self.current_ticks - self.last_ticks) / self.sampling_time
        
        # Apply low-pass filter to speed
        self.filtered_speed = self.alpha * self.filtered_speed + (1 - self.alpha) * raw_speed
        
        self.last_ticks = self.current_ticks
    
    def ticks(self):
        return self.current_ticks
    
    def speed(self):
        return self.filtered_speed

def init_logging():
    global log_dir, log_filename, log_file, log_writer
    
    # Create logs directory if it doesn't exist
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Create a new log file with timestamp in the name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"{log_dir}/robot_log_{timestamp}.csv"
    
    # Open the log file and write header
    log_file = open(log_filename, 'w', newline='')
    log_writer = csv.writer(log_file)
    
    # Write header row
    log_writer.writerow([
        'timestamp_ms',
        'tilt_angle',
        'tilt_rate',
        'position',
        'velocity',
        'pwm_output',
        'Kp',
        'Ki',
        'Kd',
        'Kp_pos',
        'Ki_pos',
        'Kd_pos'
    ])
    
    print(f"Logging data to {log_filename}")

def log_data():
    global log_writer, last_log_time
    
    current_time = time.time()
    
    # Only log at specified intervals to avoid too much data
    if current_time - last_log_time >= log_interval:
        # Get timestamp in milliseconds
        timestamp_ms = int(current_time * 1000)
        
        # Write data row
        log_writer.writerow([
            timestamp_ms,
            round(roll, 3),
            round(GyroX, 3),
            round(position, 5),
            round(velocity, 5),
            int(pwm),
            Kp,
            Ki,
            Kd,
            Kp_pos,
            Ki_pos,
            Kd_pos
        ])
        
        # Flush to ensure data is written to disk
        log_file.flush()
        
        # Update last log time
        last_log_time = current_time
    
    # Also print to console for monitoring
    if current_time - last_log_time >= 1.0:  # Print less frequently to terminal
        print_state()

def update_encoder_data():
    global position, wheelData, velocity, last_position
    
    # Reading number of ticks from encoder
    wheel_ticks = wheel_enc.read()
    
    # Calculate position in meters
    new_position = 2 * pi * wheelRadius * wheel_ticks / (wheelMotorPPR * 4)
    
    # Calculate velocity (meters per second)
    if dt > 0:
        velocity = (new_position - position) / dt
    
    # Update position
    position = new_position
    
    # Update encoder data processor
    wheelData.update(wheel_ticks, 0, 0)

def tilt_controller(roll, GyroX):
    global error_roll, error_enc, int_lean, u, pwm, sgnRoll, sgnPrevRoll
    global last_error_roll, last_error_enc
    
    sgnRoll = 1 if roll > 0 else -1
    
    error_roll = target_roll - roll
    error_enc = (target_enc - (wheel_enc.read() / 8530.0)) * 2 * pi  # 8530 encoder constant
    
    if abs(error_roll) < deadzone:
        u = 0
        pwm = abs(u)
    else:
        # Error Function Calculation
        int_lean += error_roll * dt
        int_lean = max(-20, min(int_lean, 20))  # constrain
        
        last_error_roll = error_roll
        last_error_enc = error_enc
        
        u = (Kp * error_roll) + (-Kd * GyroX) + (Ki * int_lean) + (Kd_wheel * (error_enc - last_error_enc) / dt)
        
        # Scale PWM from original 0-255 range to 0-PWM_RANGE
        pwm = abs(u) * (MAX_PWM / 255.0)
        if pwm > MAX_PWM:
            pwm = MAX_PWM
        
        pwm = pwm * sgnRoll
    
    # If Lean > 20 Degrees, stop moving
    if abs(roll) > 20:
        pwm = 0
    
    # Integral Wind-Up
    if sgnRoll != sgnPrevRoll:
        int_lean = 0
    
    sgnPrevRoll = sgnRoll

def position_controller():
    global target_roll, velocity, int_pos, last_position_error, last_position
    
    position_error = target_position - position
    
    int_pos += position_error * dt
    int_pos = max(-20, min(int_pos, 20))  # constrain
    
    # Velocity already calculated in update_encoder_data()
    
    target_roll = Kp_pos * position_error + Kd_pos * ((position_error - last_position_error) / dt) + (Ki_pos * int_pos)
    target_roll = max(-5, min(target_roll, 5))  # constrain
    
    last_position_error = position_error
    last_position = position

def write_to_motor():
    wheel_motor.set_speed(int(pwm))

def print_state():
    print(f"timestamp: {int(time.time()*1000)} ms, theta: {round(roll,2)}°, thetadot: {round(GyroX,2)}°/s, x: {round(position,5)}m, xdot: {round(velocity,5)}m/s, pwm: {int(pwm)}/{MAX_PWM}")

def sign(num):
    return -1 if num < 0 else 1

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


# Main program
if __name__ == "__main__":
    try:
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Initialize encoder and motor
        wheel_enc = Encoder(encoderPin1, encoderPin2)
        wheel_motor = HardwarePWMMotor(pwm_pin, dir_pin, PWM_RANGE, PWM_FREQUENCY)
        
        # Initialize encoder data processor
        wheelData = EncoderDataProcessor(wheelMotorPPR, 0, True, False, vel_cutoff_freq, sampling_time)
        
        # Set initial values
        sgnRoll = 0
        sgnPrevRoll = 0
        prev_time = time.time() * 1000
        deadband_sign = 1
        prev_input_sign = 1
        # Initialize time variables
        t = time.time() * 1000000  # Initial time in microseconds
        lastt = t  # Initial last time
        dt = sampling_time  # Initial dt
        
        # Main loop
        while True:
            current_time = time.time()
            
            # Check if it's time for the next sample
            if current_time - last_sample_time >= sample_interval:
                # Update sensor data
                data = mpu.update()
                
                # Display data
                print_imu_data(data)
                
                last_sample_time = current_time
            
            start_time = time.time() * 1000000  # microseconds
            
            update_encoder_data()
            
            encoder_value = wheel_enc.read()
            
            # Update time and calculate dt
            lastt = t
            t = time.time() * 1000000  # microseconds
            dt = (t - lastt) / 1000000  # seconds
            # Update sensor data
            data = mpu.update()
            
            # Display data
            print_imu_data(data)
            
            last_sample_time = current_time
            
            start_time = time.time() * 1000000  # microseconds
            
            update_encoder_data()
            
            encoder_value = wheel_enc.read()
            
            lastt = t
            t = time.time() * 1000000  # microseconds
            dt = (t - lastt) / 1000000  # seconds
            
            roll = data['angle']['roll']
            GyroX = data['gyro']['x']
            # position_controller()  # Uncomment if needed
            tilt_controller(roll, GyroX)
            
            write_to_motor()
            
            # Log data
            log_data()
            
            # Enforce loop timing
            elapsed = (time.time() * 1000000) - start_time
            if elapsed < loopTimeConstant:
                time.sleep((loopTimeConstant - elapsed) / 1000000)
    
    except KeyboardInterrupt:
        print("Program stopped by user")
    finally:
        if 'wheel_motor' in locals():
            wheel_motor.cleanup()
        if 'log_file' in locals() and log_file is not None:
            log_file.close()
            print(f"Data logged to {log_filename}")
        GPIO.cleanup()
