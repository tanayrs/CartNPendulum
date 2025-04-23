import os
import time
from motor import HardwarePWMMotor
from encoder import PiEncoder
import signal
import sys

running = True  # Flag to control the main loop

# Configure real-time scheduling
os.nice(-20)  # Highest priority
try:
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)
except PermissionError:
    print("Run with sudo for real-time scheduling")

# Pin process to CPU core 3 (isolated core)
os.sched_setaffinity(0, {3})

# Variable to store encoder ticks
current_ticks = 0

def signal_handler(sig, frame):
    global running, current_ticks
    print("\nCaught Ctrl+C, stopping collection...")
    print(f"Final encoder ticks: {current_ticks}")
    motor.cleanup()
    running = False

signal.signal(signal.SIGINT, signal_handler)

# Hardware initialization
motor = HardwarePWMMotor()

# Initialize hardware encoder
encoder = PiEncoder(pin_a=23, pin_b=24)

# Set constant motor speed
CONSTANT_SPEED = 1000

print("Starting motor at constant speed:", CONSTANT_SPEED)
print("Press Ctrl+C to stop and display the current encoder ticks")

try:
    # Start the motor
    motor.set_speed(CONSTANT_SPEED, 0)
    
    # Store initial ticks
    start_ticks = encoder.read()
    print(f"Initial encoder ticks: {start_ticks}")
    
    # Variables for tracking changes
    prev_ticks = start_ticks
    tick_count = 0
    
    while running:
        # Read current encoder position
        current_ticks = encoder.read()
        
        # Calculate change since last reading
        delta = current_ticks - prev_ticks
        
        # Print updates periodically or when significant change
        if abs(delta) > 100:
            tick_count += 1
            print(f"Update #{tick_count}: Current ticks: {current_ticks}, Delta: {delta}")
            prev_ticks = current_ticks
        
        # Small delay to avoid CPU hogging
        time.sleep(0.01)
        
except Exception as e:
    print(f"Error: {e}")
finally:
    # Clean up
    motor.stop()
    motor.cleanup()
    print("Motor stopped")
    print(f"Final encoder ticks: {current_ticks}")
