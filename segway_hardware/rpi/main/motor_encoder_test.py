import time
import matplotlib.pyplot as plt
from motor import HardwarePWMMotor
from encoder import Encoder

def motor_encoder_test():
    # Initialize hardware
    motor = HardwarePWMMotor(pwm_pin=18, dir_pin=16)
    encoder = Encoder()
    
    # Data for plotting
    x_vals = []
    encoder_vals = []
    commanded_vals = []
    
    start_time = time.time()
    
    try:
        # Test sequence
        phases = [
            (20000, "Forward", 2),
            (-20000, "Reverse", 4),
            (0, "Stopped", 6)
        ]
        
        print("Starting integrated motor/encoder test...")
        
        while time.time() - start_time < 6:  # Total test duration
            elapsed = time.time() - start_time
            
            # Update motor phase based on elapsed time
            for speed, label, trigger_time in phases:
                if elapsed < trigger_time:
                    if motor.current_speed != speed:  # Use current_speed to track state
                        print(f"Entering {label} phase")
                        motor.set_speed(speed)
                    break
            
            # Log data for plotting
            encoder_count = encoder.get_count()
            x_vals.append(elapsed)
            encoder_vals.append(encoder_count)
            commanded_vals.append(motor.current_speed)
            
        print("Test complete - saving plot...")
        
        # Save plot as an image file
        plt.figure(figsize=(10, 6))
        plt.plot(x_vals, encoder_vals, label="Encoder Count", color="blue")
        plt.plot(x_vals, commanded_vals, label="Commanded Speed", color="red", linestyle="--")
        plt.title("Motor/Encoder Test")
        plt.xlabel("Time (s)")
        plt.ylabel("Values")
        plt.legend()
        plt.grid(True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"plots/motor_encoder_test_{timestamp}.png"
        plt.savefig(filename)
        print(f"Plot saved as {filename}")

    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        motor.cleanup()
        print("Test complete")

if __name__ == "__main__":
    motor_encoder_test()

