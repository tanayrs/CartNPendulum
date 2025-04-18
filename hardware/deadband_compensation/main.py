# Add to imports
import os
import time
from motor import HardwarePWMMotor
import signal
import sys

triangle_input = -1000
sign_input = -1

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
    motor.cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Hardware initialization
motor = HardwarePWMMotor()

while True:
    try: 
        loop_start = time.monotonic()  # Monotonic clock for precision
    
        # --- Control logic (keep this under 8ms) ---
        motor.set_speed(triangle_input)
        triangle_input += sign_input

        sign_input = -sign_input if abs(triangle_input) >= 1000 else sign_input


        # -------------------------------------------
    
        # Busy-wait for precise timing
        while (time.monotonic() - loop_start) < 0.01:  # 10ms
            time.sleep(0.0001)

    except KeyboardInterrupt:
        motor.cleanup()

    except IOError as e:
        print(f"I/O Error: {e}")
        # Reset motor and flush communication buffers in case of error
        motor.stop()