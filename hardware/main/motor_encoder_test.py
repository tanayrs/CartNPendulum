# Test to check motor and encoder functionality

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
    pos_vals = []
    vel_vals = []
    
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
            pos = encoder.get_pos()
            vel = encoder.get_vel()
            
            x_vals.append(elapsed)
            angle_vals.append(angle)
            rate_vals.append(rate)
            commanded_vals.append(motor.current_speed)
            pos_vals.append(pos)
            vel_vals.append(vel)
            
        print("Test complete - saving plot...")
        
        # Create plot with two subplots
        fig, axs = plt.subplots(3, 2, figsize=(10, 8))
        ax1, ax2, ax3, ax4, ax5, ax6 = axs.flatten()
        
        # Plot angle and commanded speed
        ax1.plot(x_vals, commanded_vals, 'r--', label="Commanded Speed")
        ax1.set_title("Command")
        ax1.set_ylabel("Values")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(x_vals, angle_vals, 'b-', label="Angle (degrees)")
        ax2.set_title("Motor Angle")
        ax2.set_ylabel("Values")
        ax2.legend()
        ax2.grid(True)
        
        # Plot angular rate
        ax3.plot(x_vals, rate_vals, 'g-', label="Angular Rate (deg/s)")
        ax3.set_title("Angular Rate")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Degrees/second")
        ax3.legend()
        ax3.grid(True)

        # Plot position
        ax4.plot(x_vals, pos_vals, 'g-')
        ax4.set_title("Position (m)")
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("(m)")
        ax4.grid(True)

        # Plot velocity
        ax5.plot(x_vals, vel_vals, 'g-')
        ax5.set_title("Velocty (m/s)")
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("(m/s)")
        ax5.grid(True)

        
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

