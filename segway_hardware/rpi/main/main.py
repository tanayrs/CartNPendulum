from imu import MPU6050
from motor import HardwarePWMMotor
from encoder import EncoderAngle
from controller import TiltController
from logger import DataLogger

import time
import signal
import sys

def signal_handler(sig, frame):
    motor.cleanup()
    logger.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    # Hardware initialization
    imu = MPU6050()
    motor = HardwarePWMMotor()
    encoder = EncoderAngle(pin_a=23, pin_b=24)
    controller = TiltController(Kp=4000.0, Ki=0.0, Kd=78)
    logger = DataLogger()

    # Calibrate IMU for accurate readings
    imu.calibrate()

    logger.start()

    # Timing variables for logging every 100 ms
    log_interval = 0.1  # 100 ms interval
    last_log_time = time.time()

    try:
        while True:
            start_time = time.time()

            # Update IMU data
            imu_data = imu.update()

            # Ensure valid IMU readings
            roll_angle = imu_data['angle']['roll']
            pitch_angle = imu_data['angle']['pitch']
            gyro_rate_x = imu_data['gyro']['x']

            if roll_angle is None or pitch_angle is None:
                print("Error: IMU returned invalid data")
                continue

            # Update encoder data
            angle = encoder.get_angle()
            rate = encoder.get_rate()

            # Control calculation using roll angle and gyro rate
            control_output = controller.update(
                current_angle=roll_angle,
                gyro_rate=gyro_rate_x
            )

            # Actuation: Set motor speed based on control output
            motor.set_speed(control_output)

            # Log data every 100 ms
            current_time = time.time()
            if current_time - last_log_time >= log_interval:
                logger.log({
                    'roll': roll_angle,
                    'pitch': pitch_angle,
                    'output': control_output,
                    'speed': rate  # Log angular rate instead of raw count
                })
                last_log_time = current_time

            # Timing control to maintain consistent loop rate (100 Hz)
            elapsed = time.time() - start_time
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        motor.cleanup()
        logger.stop()

