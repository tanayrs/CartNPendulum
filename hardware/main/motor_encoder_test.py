# Test to check motor and encoder functionality
'''
Motor and Encoder Testing Script for Cart-Pendulum System

This script provides a comprehensive test of the motor and encoder hardware components
used in the Cart-Pendulum reinforcement learning platform. It runs the motor through 
a predefined sequence of movements while simultaneously capturing encoder readings.

Features:
- Tests bidirectional motor control (forward and reverse)
- Captures and plots encoder angle, angular rate, position, and velocity data
- Validates motor-encoder communication and performance
- Produces time-series visualizations of all system metrics
- Automatically saves test results as timestamped plots

Test Sequence:
1. Forward motion (20000 PWM) for 2 seconds
2. Stop for 1 second
3. Reverse motion (-20000 PWM) for 2 seconds
4. Stop for 1 second

Requirements:
- Raspberry Pi with GPIO pins connected to motor driver and encoder
- Pigpio daemon running (sudo pigpiod)
- Properly configured motor and encoder hardware
- 'motor.py' and 'encoder.py' modules in the same directory

Usage:
    python motor_encoder_test.py
    
Output:
    - Console feedback during test execution
    - A timestamped PNG file with multiple plots showing motor and encoder performance
    - Saved in the 'plots' directory
'''

# imports
import time
import matplotlib.pyplot as plt
from motor import HardwarePWMMotor
# Choose one of the implementations from above
from encoder import EncoderAngle

def directional_encoder_test():
    # Initialize hardware components
    motor = HardwarePWMMotor(pwm_pin=18, dir_pin=16)  # Create motor instance on pins 18 (PWM) and 16 (direction)
    motor.current_speed = 0  # Initialize speed tracking attribute
    encoder = EncoderAngle(pin_a=23, pin_b=24)  # Create encoder instance on pins 23 and 24
    
    # Data collection arrays for plotting
    x_vals = []  # Time values
    angle_vals = []  # Encoder angle readings
    rate_vals = []  # Angular rate readings
    commanded_vals = []  # Motor command values
    pos_vals = []  # Position values
    vel_vals = []  # Velocity values
    
    start_time = time.time()  # Mark test start time
    
    try:
        # Define test sequence: (speed, description, duration_end_time)
        phases = [
            (20000, "Forward", 2),    # Run forward for 2 seconds
            (0, "Stopped", 3),        # Stop for 1 second
            (-20000, "Reverse", 5),   # Run backward for 2 seconds
            (0, "Stopped", 6)         # Stop for final 1 second
        ]
        
        print("Starting encoder direction test...")
        
        while time.time() - start_time < 6:  # Run test for 6 seconds total
            elapsed = time.time() - start_time  # Calculate elapsed time
            
            # Determine and set motor phase based on elapsed time
            for speed, label, trigger_time in phases:
                if elapsed < trigger_time:
                    if motor.current_speed != speed:  # Only update if speed changed
                        print(f"Entering {label} phase")
                        motor.set_speed(speed)
                        motor.current_speed = speed
                    break
            
            # Collect sensor data
            angle = encoder.get_angle()  # Get current angle
            rate = encoder.get_rate()    # Get current angular rate
            pos = encoder.get_pos()      # Get current position
            vel = encoder.get_vel()      # Get current velocity
            
            # Record all data for plotting
            x_vals.append(elapsed)
            angle_vals.append(angle)
            rate_vals.append(rate)
            commanded_vals.append(motor.current_speed)
            pos_vals.append(pos)
            vel_vals.append(vel)
            
        print("Test complete - saving plot...")
        
        # Create plot with 6 subplots in 3x2 grid
        fig, axs = plt.subplots(3, 2, figsize=(10, 8))
        ax1, ax2, ax3, ax4, ax5, ax6 = axs.flatten()  # Unpack subplots for easier referencing
        
        # Plot 1: Motor command signal
        ax1.plot(x_vals, commanded_vals, 'r--', label="Commanded Speed")
        ax1.set_title("Command")
        ax1.set_ylabel("Values")
        ax1.legend()
        ax1.grid(True)

        # Plot 2: Encoder angle
        ax2.plot(x_vals, angle_vals, 'b-', label="Angle (degrees)")
        ax2.set_title("Motor Angle")
        ax2.set_ylabel("Values")
        ax2.legend()
        ax2.grid(True)
        
        # Plot 3: Angular rate
        ax3.plot(x_vals, rate_vals, 'g-', label="Angular Rate (deg/s)")
        ax3.set_title("Angular Rate")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Degrees/second")
        ax3.legend()
        ax3.grid(True)

        # Plot 4: Position
        ax4.plot(x_vals, pos_vals, 'g-')
        ax4.set_title("Position (m)")
        ax4.set_xlabel("Time (s)")
        ax4.set_ylabel("(m)")
        ax4.grid(True)

        # Plot 5: Velocity
        ax5.plot(x_vals, vel_vals, 'g-')
        ax5.set_title("Velocty (m/s)")
        ax5.set_xlabel("Time (s)")
        ax5.set_ylabel("(m/s)")
        ax5.grid(True)

        # No plot for ax6 (sixth subplot is unused)
        
        plt.tight_layout()  # Adjust subplot spacing
        timestamp = time.strftime("%Y%m%d_%H%M%S")  # Generate timestamp for unique filename
        filename = f"plots/encoder_direction_test_{timestamp}.png"
        plt.savefig(filename)  # Save figure to file
        print(f"Plot saved as {filename}")

    except Exception as e:
        print(f"Test failed: {e}")  # Error handling
    finally:
        motor.cleanup()  # Ensure motor is stopped and cleaned up
        print("Test complete")

if __name__ == "__main__":
    directional_encoder_test()  # Run the test when script is executed directly