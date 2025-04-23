# Add to imports
import os
import time
from imu import MPU6050
from motor import HardwarePWMMotor
from encoder import EncoderProcessor, PiEncoder #
from controller import TiltController
from logger import DataLogger
import signal
import sys

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
    logger.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)


# Hardware initialization
imu = MPU6050(roll_offset=0.3)
motor = HardwarePWMMotor()

# PiEncoder pins where toput
encoder = EncoderProcessor() #
controller = TiltController(Kp=5000.0, Ki=10.0, Kd=-10)
logger = DataLogger()

logger.start()

# Timing variables for logging every 100 ms
log_interval = 0.1  # 100 ms interval
last_log_time = time.time()
# Modify main loop timing section
try:
    while True:
        try: 
            loop_start = time.monotonic()  # Monotonic clock for precision
        
            # --- Control logic (keep this under 8ms) ---
            ticks = encoder.read() #
            encoder.update(ticks) #
            x = encoder.get_position_meters() #
            x_dot = encoder.get_speed_ms() #
            logger.update() #

            #---- didnt delete cuz wasn't sure
            imu_data = imu.update()
            roll_angle = imu_data['angle']['roll']
            gyro_rate_x = imu_data['gyro']['x']
            angle = encoder.get_angle()
            control_output = controller.update(roll_angle, gyro_rate_x)
            motor.set_speed(control_output)
            # -------------------------------------------

            print(f'{roll_angle=}')
        
            # Busy-wait for precise timing
            while (time.monotonic() - loop_start) < 0.01:  # 10ms
                time.sleep(0.0001)

        except KeyboardInterrupt:
            motor.cleanup()
            logger.stop()

        except IOError as e:
            print(f"I/O Error: {e}")
            # Reset motor and flush communication buffers in case of error
            motor.stop()
            encoder.encoder.steps = 0  # Reset encoder steps

except:
    pass
