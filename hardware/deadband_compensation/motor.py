# motor class file, needed to run the motor

import RPi.GPIO as GPIO
import pigpio

class HardwarePWMMotor:
    def __init__(self, pwm_pin=18, dir_pin=16, pwm_range=40000, pwm_freq=20000, static_inc = 1604, kinetic_inc=-1544, static_dec=-1786, kinetic_dec=1495):
        self.current_speed = 0
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.pwm_range = pwm_range

        self.static_inc = static_inc
        self.kinetic_inc = kinetic_inc
        self.static_dec = static_dec
        self.kinetic_dec = kinetic_dec

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(dir_pin, GPIO.OUT)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")

        self.pi.set_PWM_frequency(pwm_pin, pwm_freq)
        self.pi.set_PWM_range(pwm_pin, pwm_range)
        self.stop()

    def set_speed(self, speed, prev_vel):
        try:
            self.current_speed = speed
            if speed < 0:
                GPIO.output(self.dir_pin, GPIO.LOW)
                speed = abs(speed)
            else:
                GPIO.output(self.dir_pin, GPIO.HIGH)

            if prev_vel == 0.0 and speed > 0:
                speed += self.static_inc
            elif prev_vel != 0 and speed > 0:
                speed += self.kinetic_dec
            elif prev_vel == 0.0 and speed < 0:
                speed += self.static_dec
            elif prev_vel != 0 and speed < 0:
                speed += self.kinetic_inc

            duty_cycle = min(int(speed), self.pwm_range)
            self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)
        except Exception as e:
            print(f"Error setting motor speed: {e}")
            self.stop()

    def stop(self):
        try:
            self.pi.set_PWM_dutycycle(self.pwm_pin, 0)
        except Exception as e:
            print(f"Error stopping motor: {e}")

    def cleanup(self):
        try:
            self.stop()
            self.pi.stop()
            GPIO.cleanup()
        except Exception as e:
            print(f"Error during motor cleanup: {e}")
