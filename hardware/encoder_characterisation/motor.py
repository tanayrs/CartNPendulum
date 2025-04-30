# motor class, needed to tell motor how fast to spin, 

'''
Hardware PWM Motor Control Module

This module provides a class for precise control of DC motors using hardware PWM on Raspberry Pi.
It handles direction control and implements friction compensation for smoother motor operation
in the Cart-Pendulum reinforcement learning platform.

Features:
- Uses hardware PWM through pigpio library for precise motor control
- Supports bidirectional motor control with a dedicated direction pin
- Implements friction compensation to overcome static friction and adjust for kinetic friction
- Handles safe initialization, speed control, and shutdown of the motor

Implementation:
- Utilizes hardware PWM capabilities for precise timing and smooth operation
- Compensates for differences between static and kinetic friction
- Applies different compensation values based on current motion state and direction
- Implements safety features to ensure proper motor operation

Usage:
    motor = HardwarePWMMotor(pwm_pin=18, dir_pin=16)
    motor.set_speed(20000, prev_vel=0.0)  # Set to half speed from standstill
    # ... other operations ...
    motor.stop()  # Stop the motor
    motor.cleanup()  # Release resources when done

Requirements:
- Raspberry Pi with GPIO pins connected to motor driver
- Pigpio daemon running (sudo pigpiod)
- Motor driver compatible with PWM input for speed control and digital input for direction
- Properly configured friction compensation values for the specific motor and mechanical system

Compensation Parameters:
- static_inc: Added when starting from standstill in forward direction
- kinetic_inc: Added when already moving in forward direction
- static_dec: Added when starting from standstill in reverse direction
- kinetic_dec: Added when already moving in reverse direction
'''

# imports
import RPi.GPIO as GPIO
import pigpio

class HardwarePWMMotor:
    def __init__(self, pwm_pin=18, dir_pin=16, pwm_range=40000, pwm_freq=20000, static_inc = 1604, kinetic_inc=-1544, static_dec=-1786, kinetic_dec=1495):
        """Initialize the motor controller with GPIO pins and PWM settings.
        
        Args:
            pwm_pin: GPIO pin for PWM speed control
            dir_pin: GPIO pin for direction control
            pwm_range: Range for PWM signal (resolution)
            pwm_freq: Frequency for PWM signal in Hz
            static_inc: Compensation value for starting from standstill (forward)
            kinetic_inc: Compensation value when already moving (forward)
            static_dec: Compensation value for starting from standstill (reverse)
            kinetic_dec: Compensation value when already moving (reverse)
        """
        self.current_speed = 0
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.pwm_range = pwm_range

        self.static_inc = static_inc
        self.kinetic_inc = kinetic_inc
        self.static_dec = static_dec
        self.kinetic_dec = kinetic_dec

        # Set up GPIO mode and direction pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(dir_pin, GPIO.OUT)

        # Initialize pigpio for hardware PWM
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")

        # Configure PWM parameters
        self.pi.set_PWM_frequency(pwm_pin, pwm_freq)
        self.pi.set_PWM_range(pwm_pin, pwm_range)
        self.stop()

    def set_speed(self, speed, prev_vel):
        """Set the motor speed with compensation for friction.
        
        Args:
            speed: Target speed value
            prev_vel: Previous velocity for motion compensation
        """
        try:
            self.current_speed = speed
            # Set direction based on speed sign
            if speed < 0:
                GPIO.output(self.dir_pin, GPIO.LOW)  # Reverse direction
                speed = abs(speed)
            else:
                GPIO.output(self.dir_pin, GPIO.HIGH)  # Forward direction

            # Apply friction compensation based on motion state
            if prev_vel == 0.0 and speed > 0:
                speed += self.static_inc  # Starting from standstill (forward)
            elif prev_vel != 0 and speed > 0:
                speed += self.kinetic_dec  # Already moving (forward)
            elif prev_vel == 0.0 and speed < 0:
                speed += self.static_dec  # Starting from standstill (reverse)
            elif prev_vel != 0 and speed < 0:
                speed += self.kinetic_inc  # Already moving (reverse)

            # Ensure duty cycle is within valid range
            duty_cycle = min(int(speed), self.pwm_range)
            self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)
        except Exception as e:
            print(f"Error setting motor speed: {e}")
            self.stop()

    def stop(self):
        """Stop the motor by setting PWM to zero."""
        try:
            self.pi.set_PWM_dutycycle(self.pwm_pin, 0)
        except Exception as e:
            print(f"Error stopping motor: {e}")

    def cleanup(self):
        """Release resources and clean up GPIO."""
        try:
            self.stop()
            self.pi.stop()
            GPIO.cleanup()
        except Exception as e:
            print(f"Error during motor cleanup: {e}")