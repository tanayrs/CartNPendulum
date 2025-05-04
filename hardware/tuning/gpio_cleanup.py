'''
GPIO Cleanup Utility for Raspberry Pi
This script provides functionality to reset and clean up all GPIO pins on a Raspberry Pi,
ensuring that no pins remain active after program termination. It handles both standard
GPIO pins using RPi.GPIO and hardware PWM pins using pigpio.
'''

#!/usr/bin/env python3

# imports
import RPi.GPIO as GPIO
import pigpio
#import time

def cleanup_all_gpio():
    print("Cleaning up all GPIO pins...")
    
    # First, clean up standard GPIO pins
    try:
        GPIO.setwarnings(False)  # Suppress warnings
        GPIO.setmode(GPIO.BCM)   # Use BCM numbering scheme
        
        # Get all available GPIO pins (this may vary based on Pi model)
        # Most Raspberry Pi models have GPIO pins 2-27
        all_pins = list(range(2, 28))
        
        # Setup all pins as output
        for pin in all_pins:
            if pin in [2,3]:  # Do not close I2C SCL and SDA pins as IMU shuts off
                continue
            try:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)  # Set to LOW
                print(f"Set GPIO pin {pin} to LOW")
            except:
                pass  # Skip pins that can't be set up
        
        # Clean up GPIO
        GPIO.cleanup()
        print("Standard GPIO cleanup complete")
    except Exception as e:
        print(f"Error during standard GPIO cleanup: {e}")
    
    # Now clean up pigpio (used for hardware PWM)
    try:
        pi = pigpio.pi()
        if pi.connected:
            # PWM pins on Raspberry Pi (hardware PWM available on GPIO 12, 13, 18, 19)
            pwm_pins = [12, 13, 18, 19]
            
            for pin in pwm_pins:
                pi.set_mode(pin, pigpio.OUTPUT)
                pi.write(pin, 0)  # Set to LOW
                pi.set_PWM_dutycycle(pin, 0)  # Turn off PWM
                print(f"Reset pigpio PWM pin {pin}")
            
            pi.stop()
            print("pigpio cleanup complete")
        else:
            print("Could not connect to pigpio daemon")
    except Exception as e:
        print(f"Error during pigpio cleanup: {e}")
    
    print("All GPIO cleanup finished")

if __name__ == "__main__":
    print("Starting GPIO cleanup process...")
    cleanup_all_gpio()
    print("GPIO cleanup complete. All pins should now be reset.")