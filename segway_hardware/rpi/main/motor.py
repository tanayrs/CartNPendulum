import RPi.GPIO as GPIO
import pigpio
import time

class HardwarePWMMotor:
    def __init__(self, pwm_pin=18, dir_pin=16, 
                 pwm_range=1000000, pwm_freq=20000):
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.pwm_range = pwm_range
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(dir_pin, GPIO.OUT)
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon")
            
        self.pi.set_PWM_frequency(pwm_pin, pwm_freq)
        self.pi.set_PWM_range(pwm_pin, pwm_range)
        self.stop()

    def set_speed(self, speed):
        if speed < 0:
            GPIO.output(self.dir_pin, GPIO.LOW)
            speed = abs(speed)
        else:
            GPIO.output(self.dir_pin, GPIO.HIGH)
            
        duty_cycle = min(int(speed), self.pwm_range)
        self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)

    def stop(self):
        self.pi.set_PWM_dutycycle(self.pwm_pin, 0)

    def cleanup(self):
        self.stop()
        self.pi.stop()
        GPIO.cleanup()

