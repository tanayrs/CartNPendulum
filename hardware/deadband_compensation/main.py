# Add to imports
import os
import time
from motor import HardwarePWMMotor
from encoder import PiEncoder, EncoderProcessor
import signal
import sys
import matplotlib.pyplot as plt
import pandas as pd
import datetime

triangle_input = -1000
sign_input = -1
running = True  # Flag to control the main loop

# Configure real-time scheduling BEFORE other imports
os.nice(-20)  # Highest priority
try:
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)
except PermissionError:
    print("Run with sudo for real-time scheduling")

# Pin process to CPU core 3 (isolated core)
os.sched_setaffinity(0, {3})

def signal_handler(sig, frame):
    global running
    print("\nCaught Ctrl+C, stopping collection...")
    motor.cleanup()
    running = False  # Set flag to exit the loop instead of sys.exit()

signal.signal(signal.SIGINT, signal_handler)

# Hardware initialization
motor = HardwarePWMMotor()

# Initialize hardware encoder
encoder = PiEncoder(pin_a=23, pin_b=24)

# Initialize data processor
processor = EncoderProcessor(pulses_per_rev=2262)

ticks = []
times = []
inputs = []
velocities = []

while running:  # Use flag instead of True
    try:
        loop_start = time.monotonic()  # Monotonic clock for precision

        raw_ticks = encoder.read()
        processor.update(raw_ticks)

        ticks.append(raw_ticks)
        times.append(loop_start)
        inputs.append(triangle_input)
        velocities.append(processor.get_speed())

        # --- Control logic (keep this under 8ms) ---
        motor.set_speed(triangle_input, velocities[-1])
        triangle_input += sign_input*10

        sign_input = -sign_input if abs(triangle_input) >= 10000 else sign_input

        # -------------------------------------------

        # Busy-wait for precise timing
        while (time.monotonic() - loop_start) < 0.01 and running:
            time.sleep(0.0001)

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
    t0 = times[0]
    times_zeroed = [t - t0 for t in times]
else:
    times_zeroed = times

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

time = datetime.datetime.now()
plt.savefig(f'plots/plot_{time}.png')

print(f'Saved Figure')

df = pd.DataFrame(columns=['time','input','ticks','velocity'])
df['time'] = times
df['input'] = inputs
df['ticks'] = ticks
df['velocity'] = velocities

df.to_csv(f'./data/deadband_test_{time}.csv')

print('Saved CSV')
