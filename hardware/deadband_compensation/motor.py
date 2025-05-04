'''
Hardware PWM Motor Control

This module provides a class for precisely controlling DC motors using hardware PWM on
Raspberry Pi, with built-in compensation for motor deadband effects. 

Features:
- Hardware PWM generation using pigpio library for precise timing control
- Bidirectional motor control via separate direction pin
- Advanced deadband compensation to overcome static and kinetic friction
- Automatic adjustment of motor inputs based on current state (moving/stationary)
- Robust error handling to ensure safe motor operation

Deadband Compensation:
This implementation addresses the motor deadband problem - the non-linear response
region where small input values produce no movement due to friction. Four compensation
constants are applied:
- static_inc: Added when starting from standstill in forward direction
- kinetic_inc: Adjustment applied when already moving in forward direction
- static_dec: Added when starting from standstill in reverse direction
- kinetic_dec: Adjustment applied when already moving in reverse direction

Usage:
    # Initialize motor with optional compensation constants
    motor = HardwarePWMMotor(
        pwm_pin=18, 
        dir_pin=16,
        static_inc=1604,
        kinetic_inc=-1544,
        static_dec=-1786,
        kinetic_dec=1495
    )
    
    # Set motor speed with compensation (takes current velocity into account)
    motor.set_speed(speed_value, current_velocity)
    
    # Stop motor and release resources when done
    motor.stop()
    motor.cleanup()
'''

# imports
import RPi.GPIO as GPIO
import pigpio

class HardwarePWMMotor:
    def __init__(self, pwm_pin=18, dir_pin=16, pwm_range=40000, pwm_freq=20000, static_inc = 1604, kinetic_inc=-1544, static_dec=-1786, kinetic_dec=1495):
        # Initialize motor control parameters
        self.current_speed = 0  # Track current speed setting
        self.pwm_pin = pwm_pin  # PWM pin for speed control
        self.dir_pin = dir_pin  # Direction pin (HIGH/LOW)
        self.pwm_range = pwm_range  # Maximum PWM value
        
        # Deadband compensation values for different movement states
        self.static_inc = static_inc  # Compensation when starting from rest, forward
        self.kinetic_inc = kinetic_inc  # Compensation when already moving, forward
        self.static_dec = static_dec  # Compensation when starting from rest, reverse
        self.kinetic_dec = kinetic_dec  # Compensation when already moving, reverse

        # Configure GPIO pins
        GPIO.setmode(GPIO.BCM)  # Set GPIO mode to Broadcom SOC channel numbers
        GPIO.setup(dir_pin, GPIO.OUT)  # Set direction pin as output

        # Initialize pigpio interface for hardware PWM
        self.pi = pigpio.pi()  # Connect to pigpio daemon
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")

        # Configure PWM parameters
        self.pi.set_PWM_frequency(pwm_pin, pwm_freq)  # Set PWM frequency
        self.pi.set_PWM_range(pwm_pin, pwm_range)  # Set PWM range
        self.stop()  # Ensure motor starts in stopped state

    def set_speed(self, speed, prev_vel):
        try:
            self.current_speed = speed  # Update current speed tracker
            if speed < 0:
                GPIO.output(self.dir_pin, GPIO.LOW)  # Set direction to reverse
                speed = abs(speed)  # Convert to positive for PWM calculation
            else:
                GPIO.output(self.dir_pin, GPIO.HIGH)  # Set direction to forward

            # Apply deadband compensation based on movement state
            if prev_vel == 0.0 and speed > 0:
                speed += self.static_inc  # Starting from rest, moving forward
            elif prev_vel != 0 and speed > 0:
                speed += self.kinetic_dec  # Already moving, continuing forward
            elif prev_vel == 0.0 and speed < 0:
                speed += self.static_dec  # Starting from rest, moving backward
            elif prev_vel != 0 and speed < 0:
                speed += self.kinetic_inc  # Already moving, continuing backward

            # Apply PWM signal with duty cycle capped at maximum range
            duty_cycle = min(int(speed), self.pwm_range)
            self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)
        except Exception as e:
            print(f"Error setting motor speed: {e}")
            self.stop()  # Safety measure: stop on error

    def stop(self):
        try:
            self.pi.set_PWM_dutycycle(self.pwm_pin, 0)  # Set PWM to zero to stop motor
        except Exception as e:
            print(f"Error stopping motor: {e}")

    def cleanup(self):
        try:
            self.stop()  # Stop motor first
            self.pi.stop()  # Release pigpio resources
            GPIO.cleanup()  # Clean up GPIO settings
        except Exception as e:
            print(f"Error during motor cleanup: {e}")