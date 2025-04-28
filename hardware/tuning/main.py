# main.py
import numpy as np
import os
import sys
import time
from imu import MPU6050
from motor import HardwarePWMMotor
from encoder import EncoderProcessor, PiEncoder
from controller import TiltController
from logger import DataLogger
import signal
import datetime
import gymnasium
from gymnasium.envs.registration import register

# Add the path to your custom environment
base_dir = '/home/cartpend/CartNPendulum'
sys.path.append(base_dir)

# Configure real-time scheduling BEFORE other imports
os.nice(-20)
try:
    param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
    os.sched_setscheduler(0, os.SCHED_FIFO, param)
except PermissionError:
    print("Run with sudo for real-time scheduling")

os.sched_setaffinity(0, {3})

def signal_handler(sig, frame):
    motor.cleanup()
    logger.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# Hardware initialization
imu = MPU6050(roll_offset=0)
motor = HardwarePWMMotor()
encoder = PiEncoder(pin_a=23, pin_b=24)
processor = EncoderProcessor(pulses_per_rev=2262)
controller = TiltController(Kp=3000.0, Ki=10.0, Kd=-10)

# Register the custom environment
try:
    register(
        id="CustomCartPole-v1",
        entry_point="python.environment.custom_cartpole:CartPoleEnv",  # Adjust path if needed
    )
except gymnasium.error.Error as e:
    if "already registered" in str(e):
        pass
    else:
        raise e

ppo_model_path = os.path.join(base_dir, 'Training', 'Saved Models', 'PPO_model_jia.zip')

from controller_rl import HardwareModelAgent

controller_rl = HardwareModelAgent(
    model_type='PPO',
    model_path=ppo_model_path,
    env_name='CustomCartPole-v1'
)

logger = DataLogger()
logger.start()

log_interval = 0.1
last_log_time = time.time()

try:
    while True:
        try:
            loop_start = time.monotonic()

            # --- Control logic (keep this under 8ms) ---
            ticks = encoder.read()
            processor.update(ticks)
            x = processor.get_position_meters()
            x_dot = processor.get_speed_ms()
            imu_data = imu.update()
            roll_angle = imu_data['angle']['roll']
            gyro_rate_x = imu_data['gyro']['x']

            dts = datetime.datetime.now()
            theta_rad = np.pi * roll_angle / 180
            control_output = controller_rl.control([x, x_dot, theta_rad, gyro_rate_x])
            dte = datetime.datetime.now()
            time4ctrlout = dte - dts
            s = (time4ctrlout.total_seconds())
            ms = round(s * 1000)

            data = {
                'x': x,
                'x_dot': x_dot,
                'theta': roll_angle,
                'theta_dot': gyro_rate_x,
                'control_output': control_output
            }

            motor.set_speed(control_output, x_dot)
            logger.log(data)
            print(f'{roll_angle=}')

            while (time.monotonic() - loop_start) < 0.01:
                time.sleep(0.0001)

        except KeyboardInterrupt:
            motor.cleanup()
            logger.stop()
            break

        except IOError as e:
            print(f"I/O Error: {e}")
            motor.stop()

except Exception as e:
    print(f"Unexpected error: {e}")
    motor.cleanup()
    logger.stop()
