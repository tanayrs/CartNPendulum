# This file finds the max motor speed and maximum input PWM value

'''
Motor Maximum Speed Characterization

This script systematically tests the motor's speed response to increasing PWM inputs to
determine the maximum achievable velocity and identify potential non-linearities in the
motor response curve.

Features:
- Real-time process scheduling for precise timing measurements
- Gradually increasing PWM input values to test motor response across full range
- High-frequency sampling of encoder position and velocity
- Synchronized collection of time, input, encoder and velocity data
- CPU core isolation for consistent timing performance
- Graceful signal handling to ensure proper hardware shutdown
- Visualization of motor speed response characteristics

Purpose:
This characterization helps establish:
1. The maximum achievable speed of the motor-encoder system
2. The relationship between PWM input values and resulting velocities
3. Any non-linearities in the motor response curve beyond the deadband region
4. Potential saturation points where increasing inputs no longer increase speed

Usage:
    sudo python find_max_speed.py
    
    # Run the test until desired maximum speed is reached
    # Press Ctrl+C to stop data collection and save results

Output:
    - Time-series plot showing motor input and resulting velocity
    - CSV file with all recorded data saved to data/ directory
    - Visual indication of maximum achieved velocity

Requirements:
    - Raspberry Pi with properly connected motor and encoder
    - Root privileges for real-time scheduling (run with sudo)
    - Pre-created 'data' directory
    - Properly configured motor.py and encoder.py modules
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

# Control variables
triangle_input = -1000     # Initial PWM input value 
sign_input = -1            # Direction of PWM change (negative)
running = True             # Flag to control the main loop

# Configure real-time scheduling BEFORE other imports
os.nice(-20)               # Set highest process priority
try:
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)  # Set FIFO scheduling policy
except PermissionError:
    print("Run with sudo for real-time scheduling")

# Pin process to CPU core 3 (isolated core) for better real-time performance
os.sched_setaffinity(0, {3})

def signal_handler(sig, frame):
    global running
    print("\nCaught Ctrl+C, stopping collection...")
    motor.cleanup()        # Ensure motor resources are properly released
    running = False        # Signal main loop to exit gracefully

signal.signal(signal.SIGINT, signal_handler)  # Register interrupt handler

# Hardware initialization
motor = HardwarePWMMotor()  # Initialize PWM motor controller

# Initialize hardware encoder
encoder = PiEncoder(pin_a=23, pin_b=24)  # Set up encoder on GPIO pins 23 and 24

# Initialize data processor for converting raw encoder ticks
processor = EncoderProcessor(pulses_per_rev=2262)  # Configure with encoder resolution

# Data collection arrays
ticks = []                 # Store raw encoder ticks
times = []                 # Store timestamps
inputs = []                # Store PWM input values
velocities = []            # Store calculated velocities

while running:
    try:
        loop_start = time.monotonic()  # High precision timestamp for this loop iteration

        # Read and process encoder data
        raw_ticks = encoder.read()
        processor.update(raw_ticks)

        # Store data points
        ticks.append(raw_ticks)
        times.append(loop_start)
        inputs.append(triangle_input)
        velocities.append(processor.get_speed())

        # Set motor speed and update input for next iteration
        motor.set_speed(triangle_input)
        triangle_input += sign_input*10  # Gradually change PWM value by 10 units each iteration

        # Maintain consistent 10ms loop timing using busy-wait for precision
        while (time.monotonic() - loop_start) < 0.01 and running:
            time.sleep(0.0001)  # Small sleep to reduce CPU usage while busy-waiting

    except KeyboardInterrupt:
        # Fallback interrupt handler
        motor.cleanup()
        running = False
        break

    except IOError as e:
        print(f"I/O Error: {e}")
        motor.stop()  # Safety stop in case of I/O errors

print("Creating and saving plots...")

# Normalize time values to start from zero
if len(times) > 0:
    t0 = times[0]
    times_zeroed = [t - t0 for t in times]
else:
    times_zeroed = times

# Create visualization plots
plt.figure(figsize=(10, 6))
plt.subplot(2,1,1)
plt.scatter(times_zeroed, inputs, label='Motor Input', s=10)
plt.ylabel('Input')
plt.grid()

plt.subplot(2,1,2)
plt.scatter(times_zeroed, velocities, label='Velocity', s=10)
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (deg/s)')
plt.title('Velocity vs Time')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Save plot with timestamp in filename
time = datetime.datetime.now()
plt.savefig(f'plots/plot_{time}.png')
print(f'Saved Figure')

# Save collected data to CSV file
df = pd.DataFrame(columns=['time','input','ticks','velocity'])
df['time'] = times
df['input'] = inputs
df['ticks'] = ticks
df['velocity'] = velocities
df.to_csv(f'./data/deadband_test_{time}.csv')
print('Saved CSV')