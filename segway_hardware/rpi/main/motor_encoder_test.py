import time
import matplotlib.pyplot as plt
from motor import HardwarePWMMotor
# Choose one of the implementations from above
from encoder import EncoderAngle

def directional_encoder_test():
    # Initialize hardware
    motor = HardwarePWMMotor(pwm_pin=18, dir_pin=16)
    motor.current_speed = 0  # Add this attribute if not already present
    encoder = EncoderAngle(pin_a=23, pin_b=24)
    
    # Data for plotting
    x_vals = []
    angle_vals = []
    rate_vals = []
    commanded_vals = []
    
    start_time = time.time()
    
    try:
        # Test sequence
        phases = [
            (20000, "Forward", 2),
            (0, "Stopped", 3),
            (-20000, "Reverse", 5),
            (0, "Stopped", 6)
        ]
        
        print("Starting encoder direction test...")
        
        while time.time() - start_time < 6:
            elapsed = time.time() - start_time
            
            # Update motor phase
            for speed, label, trigger_time in phases:
                if elapsed < trigger_time:
                    if motor.current_speed != speed:
                        print(f"Entering {label} phase")
                        motor.set_speed(speed)
                        motor.current_speed = speed
                    break
            
            # Update data
            angle = encoder.get_angle()
            rate = encoder.get_rate()
            
            x_vals.append(elapsed)
            angle_vals.append(angle)
            rate_vals.append(rate)
            commanded_vals.append(motor.current_speed)
            
        print("Test complete - saving plot...")
        
        # Create plot with two subplots
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Plot angle and commanded speed
        ax1.plot(x_vals, angle_vals, 'b-', label="Angle (degrees)")
        ax1.plot(x_vals, commanded_vals, 'r--', label="Commanded Speed")
        ax1.set_title("Motor Angle and Command")
        ax1.set_ylabel("Values")
        ax1.legend()
        ax1.grid(True)
        
        # Plot angular rate
        ax2.plot(x_vals, rate_vals, 'g-', label="Angular Rate (deg/s)")
        ax2.set_title("Angular Rate")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Degrees/second")
        ax2.legend()
        ax2.grid(True)
        
        plt.tight_layout()
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"plots/encoder_direction_test_{timestamp}.png"
        plt.savefig(filename)
        print(f"Plot saved as {filename}")

    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        motor.cleanup()
        print("Test complete")

if __name__ == "__main__":
    directional_encoder_test()

