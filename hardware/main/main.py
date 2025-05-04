'''
Main Control Script

This script implements real-time control of a physical Segway system 
using reinforcement learning. It integrates hardware components, sensor processing, and RL-based control to balance an inverted pendulum on a moving cart.

Features:
- Real-time control loop running at 100Hz (10ms period)
- Hardware interface with IMU sensor, motor controller, and quadrature encoder
- Reinforcement learning agent deployment for pendulum balancing
- Real-time scheduling with process priority optimization
- Data logging of system states and control outputs
- Graceful shutdown with signal handling

System Components:
- MPU6050 IMU sensor for pendulum angle measurement
- DC motor with encoder for cart position control
- Raspberry Pi for real-time processing and control
- RL agent trained in simulation and deployed to hardware

Control Flow:
1. Read sensor data (encoder position, IMU angle)
2. Process sensor data to obtain state variables (x, x_dot, theta, theta_dot)
3. Feed state to RL controller to compute control action
4. Apply control action to motor
5. Log data for analysis
6. Maintain precise 10ms timing for control loop

Requirements:
- Raspberry Pi with GPIO pins connected to hardware
- Python 3.7+ with gymnasium, numpy
- Pre-trained RL model (PPO or DQN)
- Hardware components (IMU, motor, encoder) properly configured
- Run with sudo for real-time scheduling permissions

Usage:
    sudo python main.py
    
Safety:
    Press Ctrl+C for safe shutdown that properly stops the motor
'''

# imports
import numpy as np
import os
import sys
import time
from imu import MPU6050  # IMU sensor for angular measurements
from motor import HardwarePWMMotor  # Motor control interface
from encoder import EncoderProcessor, PiEncoder  # Position tracking
from controller import TiltController  # PID controller for tilt
from logger import DataLogger  # Data logging utility
import signal
import datetime
import gymnasium
from gymnasium.envs.registration import register  # For registering custom environment

# Add the path to your custom environment
base_dir = '/home/cartpend/CartNPendulum'  # Adjust to your project root
sys.path.append(base_dir)  # Add project root to Python path

# Configure real-time scheduling BEFORE other imports
os.nice(-20)  # Highest priority for process nice value
try:
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)  # Set FIFO real-time scheduling
except PermissionError:
    print("Run with sudo for real-time scheduling")

# Pin process to CPU core 3 (isolated core) to reduce scheduling jitter
os.sched_setaffinity(0, {3})

def signal_handler(sig, frame):
    # Clean shutdown on CTRL+C
    motor.cleanup()
    logger.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Hardware initialization
imu = MPU6050(roll_offset=3.5)
motor = HardwarePWMMotor()
# Initialize hardware encoder
encoder = PiEncoder(pin_a=23, pin_b=24)
# Initialize data processor
processor = EncoderProcessor(pulses_per_rev=2262)
controller = TiltController(Kp=3000.0, Ki=10.0, Kd=-10)

# Register the custom environment
try:
    register(
        id="CustomCartPole-v1",
        entry_point="python.environment.custom_cartpole:CartPoleEnv",  # Path to environment class
    )
except gymnasium.error.Error as e:
    # If the environment has already been registered, ignore the error
    if "already registered" in str(e):
        pass
    else:
        raise e

# Use absolute paths for model files
# ppo_model_path = os.path.join(base_dir, 'Training', 'Saved Models', 'PPO_model_jia.zip')
# dqn_model_path = os.path.join(base_dir, 'Training', 'Saved Models', 'DQN_model.zip')


# Use hardware trained model 
model_path = os.path.join(base_dir, 'hardware', 'tuning','hardware_trained_models', 'final_hardware_model.zip')

from controller_rl import HardwareModelAgent
controller_rl = HardwareModelAgent(model_type='DQN', model_path=model_path, env_name='CustomCartPole-v1')

logger = DataLogger()  # Initialize data logger
logger.start()  # Start logging thread

# Timing variables for logging every 100 ms
log_interval = 0.1  # 100 ms interval
last_log_time = time.time()

# Main control loop
try:
    while True:
        try:
            loop_start = time.monotonic()  # Monotonic clock for precise timing
            
            # --- Control logic (keep this under 8ms for real-time performance) ---
            ticks = encoder.read()  # Read raw encoder ticks
            processor.update(ticks)  # Process encoder data
            x = processor.get_position_meters()  # Get cart position in meters
            x_dot = processor.get_speed_ms()  # Get cart velocity in m/s
            
            imu_data = imu.update()  # Get latest IMU data
            roll_angle = imu_data['angle']['roll']  # Pendulum angle in degrees
            gyro_rate_x = imu_data['gyro']['x']  # Angular velocity in deg/s
            
            dts = datetime.datetime.now()
            control_output = controller.update(roll_angle, gyro_rate_x)
            theta_rad = np.pi*roll_angle/180
            thetadot_rad = np.pi*gyro_rate_x/180
            #control_output = controller_rl.control([x, x_dot, theta_rad, thetadot_rad])
            dte = datetime.datetime.now()
            time4ctrlout = dte-dts

            s = (time4ctrlout.total_seconds())
            ms = round(s*1000)  # Control computation time in milliseconds
            
            data = {
                'x': x,
                'x_dot': x_dot,
                'theta': roll_angle,
                'theta_dot': gyro_rate_x,
                'control_output': control_output
            }  # Prepare data for logging
            
            motor.set_speed(control_output, x_dot)  # Apply control to motor
            logger.log(data)  # Log state and control data
            # -------------------------------------------
            
            print(f'{roll_angle=}')  # Print current angle for monitoring
            
            # Busy-wait for precise timing - ensures 10ms control loop
            while (time.monotonic() - loop_start) < 0.01:  # 10ms (100Hz control rate)
                time.sleep(0.0001)  # Small sleep to prevent CPU hogging
                
        except KeyboardInterrupt:
            motor.cleanup()  # Safe shutdown of motor
            logger.stop()  # Stop data logging
            break
        except IOError as e:
            print(f"I/O Error: {e}")
            # Reset motor and flush communication buffers in case of error
            motor.stop()
            
except Exception as e:
    print(f"Unexpected error: {e}")
    motor.cleanup()  # Ensure motor is stopped on any error
    logger.stop()  # Ensure logging is stopped