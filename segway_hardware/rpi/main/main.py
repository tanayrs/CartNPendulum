from imu import MPU6050
from motor import HardwarePWMMotor
from encoder import Encoder
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
    encoder = Encoder()
    controller = TiltController(Kp=25.0, Ki=0.0, Kd=0.5)
    logger = DataLogger()
    
    # Calibration
    #imu.calibrate()
    logger.start()
    
    try:
        while True:
            start_time = time.time()
            
            # Sensor reading
            imu_data = imu.update()
            encoder_count = encoder.get_count()
            
            # Control calculation
            control_output = controller.update(
                current_angle=imu_data['angle']['roll'],
                gyro_rate=imu_data['gyro']['x']
            )
            
            # Actuation
            motor.set_speed(control_output)
            
            # Logging
            logger.log({
                'roll': imu_data['angle']['roll'],
                'pitch': imu_data['angle']['pitch'],
                'output': control_output,
                'speed': encoder_count
            })
            
            # Timing control
            elapsed = time.time() - start_time
            if elapsed < 0.01:
                time.sleep(0.01 - elapsed)
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        motor.cleanup()
        logger.stop()

