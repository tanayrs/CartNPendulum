# If you delete, motor no go
'''
Hardware PWM Motor Control Module for Raspberry Pi

This module provides a class for precise control of DC motors using hardware PWM on Raspberry Pi.
It handles direction control and implements friction compensation for smoother motor operation.

Features:
- Uses hardware PWM through pigpio library for precise motor control
- Supports bidirectional motor control with a dedicated direction pin
- Implements friction compensation to overcome static friction and adjust for kinetic friction
- Handles safe initialization, speed control, and shutdown of the motor

Requirements:
- Raspberry Pi with GPIO pins connected to motor driver
- Pigpio daemon running (sudo pigpiod)
- Motor driver compatible with PWM input for speed control and digital input for direction

Usage:
    motor = HardwarePWMMotor(pwm_pin=18, dir_pin=16)
    motor.set_speed(20000, prev_vel=0.0)  # Set to half speed from standstill
    # ... other operations ...
    motor.stop()  # Stop the motor
    motor.cleanup()  # Release resources when done
'''

# imports
import RPi.GPIO as GPIO
import pigpio

class HardwarePWMMotor:
    def __init__(self, pwm_pin=18, dir_pin=16, pwm_range=40000, pwm_freq=20000, static_inc = 1604, kinetic_inc=-1544, static_dec=-1786, kinetic_dec=1495):
        # Current speed of the motor
        self.current_speed = 0
        # Pin for PWM signal
        self.pwm_pin = pwm_pin
        # Pin for direction control
        self.dir_pin = dir_pin
        # Maximum range for PWM
        self.pwm_range = pwm_range

        # Compensation values for different friction conditions
        # Added to PWM when starting from standstill in forward direction
        self.static_inc = static_inc
        # Added to PWM when already moving in forward direction
        self.kinetic_inc = kinetic_inc
        # Added to PWM when starting from standstill in reverse direction
        self.static_dec = static_dec
        # Added to PWM when already moving in reverse direction
        self.kinetic_dec = kinetic_dec

        # Set up GPIO mode
        GPIO.setmode(GPIO.BCM)
        # Configure direction pin as output
        GPIO.setup(dir_pin, GPIO.OUT)

        # Initialize pigpio interface
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")

        # Configure PWM frequency and range
        self.pi.set_PWM_frequency(pwm_pin, pwm_freq)
        self.pi.set_PWM_range(pwm_pin, pwm_range)
        # Initially stop the motor
        self.stop()

    def set_speed(self, speed, prev_vel):
        try:
            # Store the requested speed
            self.current_speed = speed
            # Set direction based on speed sign
            if speed < 0:
                GPIO.output(self.dir_pin, GPIO.LOW)  # Reverse direction
                speed = abs(speed)
            else:
                GPIO.output(self.dir_pin, GPIO.HIGH)  # Forward direction

            # Apply appropriate friction compensation based on previous velocity
            if prev_vel == 0.0 and speed > 0:
                speed += self.static_inc  # Starting from standstill, going forward
            elif prev_vel != 0 and speed > 0:
                speed += self.kinetic_dec  # Already moving, going forward
            elif prev_vel == 0.0 and speed < 0:
                speed += self.static_dec  # Starting from standstill, going backward
            elif prev_vel != 0 and speed < 0:
                speed += self.kinetic_inc  # Already moving, going backward

            # Ensure duty cycle is within valid range
            duty_cycle = min(int(speed), self.pwm_range)
            # Set the PWM duty cycle
            self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)
        except Exception as e:
            print(f"Error setting motor speed: {e}")
            self.stop()  # Safety: stop motor on error

    def stop(self):
        try:
            # Set PWM to zero to stop the motor
            self.pi.set_PWM_dutycycle(self.pwm_pin, 0)
        except Exception as e:
            print(f"Error stopping motor: {e}")

    def cleanup(self):
        try:
            # Stop the motor first
            self.stop()
            # Release pigpio resources
            self.pi.stop()
            # Clean up GPIO pins
            GPIO.cleanup()
        except Exception as e:
            print(f"Error during motor cleanup: {e}")