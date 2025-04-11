#!/usr/bin/env python3

import smbus
import math
import time
import RPi.GPIO as GPIO
from time import sleep
import threading
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

# MPU6050 Initialization
MPU_ADDR = 0x68  # MPU6050 I2C address
bus = smbus.SMBus(1)  # for Raspberry Pi revision 2 and later, use bus 1

# Wake up the MPU6050
bus.write_byte_data(MPU_ADDR, 0x6B, 0)

# Initialize variables
AccX, AccY, AccZ = 0.0, 0.0, 0.0
GyroX, GyroY, GyroZ = 0.0, 0.0, 0.0
accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ = 0.0, 0.0, 0.0, 0.0, 0.0
roll, pitch, yaw = 0.0, 0.0, 0.0
AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ = 0.0, 0.0, 0.0, 0.0, 0.0
elapsedTime, currentTime, previousTime = 0.0, 0.0, 0.0
t, lastt, dt = 0.0, 0.0, 0.0

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
PWM_RANGE = 1000000    # Maximum resolution (1M steps)

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

# Class for encoder pulse counting
class Encoder:
    def __init__(self, pin_a, pin_b):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.value = 0
        self.last_encoded = 0
        
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self.update)
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

# Class for sensor filtering
class EncoderDataProcessor:
    def __init__(self, ppr, steer_offset, invert_direction, invert_steer, vel_cutoff_freq, sampling_time):
        self.ppr = ppr
        self.steer_offset = steer_offset
        self.invert_direction = invert_direction
        self.invert_steer = invert_steer
        self.vel_cutoff_freq = vel_cutoff_freq
        self.sampling_time = sampling_time
        
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

def read_imu():
    global AccX, AccY, AccZ, GyroX, GyroY, GyroZ
    global accAngleX, accAngleY, gyroAngleX, gyroAngleY, yaw, roll, pitch
    global elapsedTime, currentTime, previousTime, sgnRoll
    
    # Read accelerometer data
    acc_data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 6)
    AccX = (acc_data[0] << 8 | acc_data[1]) / 16384.0
    AccY = (acc_data[2] << 8 | acc_data[3]) / 16384.0
    AccZ = (acc_data[4] << 8 | acc_data[5]) / 16384.0
    
    # Calculate Roll and Pitch from accelerometer data
    accAngleX = (math.atan(AccY / math.sqrt(AccX**2 + AccZ**2)) * 180 / pi) + 1.68
    accAngleY = (math.atan(-1 * AccX / math.sqrt(AccY**2 + AccZ**2)) * 180 / pi) + 1.35
    
    # Read gyroscope data
    previousTime = currentTime
    currentTime = time.time() * 1000  # milliseconds
    elapsedTime = (currentTime - previousTime) / 1000  # seconds
    
    gyro_data = bus.read_i2c_block_data(MPU_ADDR, 0x43, 6)
    GyroX = (gyro_data[0] << 8 | gyro_data[1]) / 131.0 + 3.52
    GyroY = (gyro_data[2] << 8 | gyro_data[3]) / 131.0 + 1.46
    GyroZ = (gyro_data[4] << 8 | gyro_data[5]) / 131.0 + 0.08
    
    # Calculate angles from gyroscope
    gyroAngleX += GyroX * elapsedTime
    gyroAngleY += GyroY * elapsedTime
    yaw += GyroZ * elapsedTime
    
    # Complementary filter
    roll = (0.98 * gyroAngleX) + (0.02 * accAngleX)
    pitch = (0.98 * gyroAngleY) + (0.02 * accAngleY)
    
    gyroAngleX = roll
    gyroAngleY = pitch
    
    roll -= roll_offset
    sgnRoll = 1 if roll > 0 else -1

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

def calculate_imu_error():
    global AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ
    
    c = 0
    
    # Read accelerometer values 200 times
    while c < 200:
        acc_data = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 6)
        AccX = (acc_data[0] << 8 | acc_data[1]) / 16384.0
        AccY = (acc_data[2] << 8 | acc_data[3]) / 16384.0
        AccZ = (acc_data[4] << 8 | acc_data[5]) / 16384.0
        
        AccErrorX += (math.atan(AccY / math.sqrt(AccX**2 + AccZ**2)) * 180 / pi)
        AccErrorY += (math.atan(-1 * AccX / math.sqrt(AccY**2 + AccZ**2)) * 180 / pi)
        
        c += 1
    
    # Divide the sum by 200 to get the error value
    AccErrorX /= 200
    AccErrorY /= 200
    
    c = 0
    
    # Read gyro values 200 times
    while c < 200:
        gyro_data = bus.read_i2c_block_data(MPU_ADDR, 0x43, 6)
        GyroX = (gyro_data[0] << 8 | gyro_data[1]) / 131.0
        GyroY = (gyro_data[2] << 8 | gyro_data[3]) / 131.0
        GyroZ = (gyro_data[4] << 8 | gyro_data[5]) / 131.0
        
        GyroErrorX += GyroX
        GyroErrorY += GyroY
        GyroErrorZ += GyroZ
        
        c += 1
    
    # Divide the sum by 200 to get the error value
    GyroErrorX /= 200
    GyroErrorY /= 200
    GyroErrorZ /= 200
    
    print(f"AccErrorX: {AccErrorX}")
    print(f"AccErrorY: {AccErrorY}")
    print(f"GyroErrorX: {GyroErrorX}")
    print(f"GyroErrorY: {GyroErrorY}")
    print(f"GyroErrorZ: {GyroErrorZ}")

def tilt_controller():
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
        
        # Initialize logging
        init_logging()
        
        # Wake up the MPU6050
        calculate_imu_error()
        
        print(f"Starting balancing loop with PWM range: {PWM_RANGE}, PWM frequency: {PWM_FREQUENCY}Hz")
        
        # Main loop
        while True:
            start_time = time.time() * 1000000  # microseconds
            
            read_imu()
            update_encoder_data()
            
            encoder_value = wheel_enc.read()
            
            lastt = t
            t = time.time() * 1000000  # microseconds
            dt = (t - lastt) / 1000000  # seconds
            
            # position_controller()  # Uncomment if needed
            tilt_controller()
            
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
