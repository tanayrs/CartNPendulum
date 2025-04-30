'''
What is motor deadband?
    For some voltages given into the motor, it does not rotate, after which the speed increases linearly with voltage. This is the deadband of the motor. 
    When we are doing control over this, we want to add this deadband to the input voltage, so that we can get a linear response from the motor.
'''

'''
Motor Deadband Characterization

This script measures and characterizes the motor deadband - the range of input values 
where the motor doesn't respond due to static friction. It applies a triangular pattern 
of input values to the motor while simultaneously recording the encoder response, 
enabling precise identification of deadband boundaries.

What is motor deadband?
    For some voltages given into the motor, it does not rotate, after which the speed 
    increases linearly with voltage. This is the deadband of the motor. When doing control, 
    we need to compensate for this deadband to achieve a linear motor response.

Features:
- Real-time process scheduling for precise timing measurements
- Applies a triangular waveform input to the motor (gradually increasing then decreasing)
- High-frequency sampling of encoder position and velocity
- Records synchronized time, input, encoder tick, and velocity data
- CPU core isolation for consistent timing performance
- Graceful signal handling to ensure proper hardware shutdown
- Generates visualizations of motor response characteristics
- Saves raw data for further analysis and compensation constant calculation

Purpose:
This characterization helps establish the exact voltage thresholds where:
1. The motor starts moving from standstill (static friction threshold)
2. The motor stops moving when decelerating (kinetic friction threshold)
The data collected will be used as input to the find_constants_10.py script to 
calculate compensation constants for the motor control system.

Usage:
    sudo python main.py
    
    # Press Ctrl+C to stop data collection and save results

Output:
    - Time-series plot of motor input and velocity saved to plots/
    - CSV file with all recorded data saved to data/
    - Console feedback during execution

Requirements:
    - Raspberry Pi with properly connected motor and encoder
    - Root privileges for real-time scheduling (run with sudo)
    - Pre-created 'plots' and 'data' directories
'''

# imports
import os
import time
from motor import HardwarePWMMotor
from encoder import PiEncoder, EncoderProcessor
import signal
import matplotlib.pyplot as plt
import pandas as pd
import datetime

triangle_input = -1000  # Initial motor input value
sign_input = -1  # Direction for triangle input changes
running = True  # Flag to control the main loop

# Configure real-time scheduling BEFORE other imports
os.nice(-20)  # Highest priority
try:
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)  # Set to FIFO scheduling policy
except PermissionError:
    print("Run with sudo for real-time scheduling")

# Pin process to CPU core 3 (isolated core)
os.sched_setaffinity(0, {3})  # Restricts execution to core 3 only

def signal_handler(sig, frame):
    global running
    print("\nCaught Ctrl+C, stopping collection...")
    motor.cleanup()  # Clean up motor resources
    running = False  # Set flag to exit the loop instead of sys.exit()

signal.signal(signal.SIGINT, signal_handler)  # Register Ctrl+C handler

# Hardware initialization
#motor = HardwarePWMMotor(static_inc=0, kinetic_inc=0, static_dec=0, kinetic_dec=0)
motor = HardwarePWMMotor()  # Initialize motor with default parameters

# Initialize hardware encoder
encoder = PiEncoder(pin_a=23, pin_b=24)  # Configure GPIO pins for encoder

# Initialize data processor
processor = EncoderProcessor(pulses_per_rev=2262)  # Set encoder resolution

# Data collection arrays
ticks = []  # Store raw encoder ticks
times = []  # Store timestamps
inputs = []  # Store motor input values
velocities = []  # Store calculated velocities

while running:  # Use flag instead of True
    try:
        loop_start = time.monotonic()  # Monotonic clock for precision timing

        raw_ticks = encoder.read()  # Read current encoder position
        processor.update(raw_ticks)  # Process raw encoder data

        # Store data points
        ticks.append(raw_ticks)
        times.append(loop_start)
        inputs.append(triangle_input)
        velocities.append(processor.get_speed())

        # --- Control logic (keep this under 8ms) ---
        motor.set_speed(triangle_input, velocities[-1])  # Apply motor input
        triangle_input += sign_input*10  # Increment/decrement input value

        # Reverse direction if input reaches limits (creates triangle wave)
        sign_input = -sign_input if abs(triangle_input) >= 10000 else sign_input

        # -------------------------------------------

        # Busy-wait for precise timing (10ms control loop)
        while (time.monotonic() - loop_start) < 0.01 and running:
            time.sleep(0.0001)  # Small sleep to reduce CPU usage

    except KeyboardInterrupt:
        # This is a fallback in case signal handler doesn't catch it
        motor.cleanup()
        running = False
        break

    except IOError as e:
        print(f"I/O Error: {e}")
        # Reset motor and flush communication buffers in case of error
        motor.stop()

print("Creating and saving plots...")

if len(times) > 0:
    t0 = times[0]  # Get start time
    times_zeroed = [t - t0 for t in times]  # Zero-reference all timestamps
else:
    times_zeroed = times

# Create visualization
plt.figure(figsize=(10, 6))
plt.subplot(2,1,1)
plt.scatter(times_zeroed, inputs, label='Motor Input', s=10)  # Plot motor inputs
plt.ylabel('Input')
plt.grid()

plt.subplot(2,1,2)
plt.scatter(times_zeroed, velocities, label='Velocity', s=10)  # Plot velocities
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (deg/s)')
plt.title('Velocity vs Time')
plt.legend()
plt.grid(True)
plt.tight_layout()

time = datetime.datetime.now()  # Get current time for filename
plt.savefig(f'plots/plot_{time}.png')  # Save figure with timestamp

print(f'Saved Figure')

# Save collected data to CSV
df = pd.DataFrame(columns=['time','input','ticks','velocity'])
df['time'] = times
df['input'] = inputs
df['ticks'] = ticks
df['velocity'] = velocities

df.to_csv(f'./data/deadband_test_{time}.csv')  # Save data with timestamp

print('Saved CSV')