import RPi.GPIO as GPIO
import math

class Encoder:
    def __init__(self, pin_a=23, pin_b=24):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.value = 0
        self.last_encoded = 0
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=self.update)
        GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=self.update)

    def update(self, channel):
        a = GPIO.input(self.pin_a)
        b = GPIO.input(self.pin_b)
        encoded = (a << 1) | b
        diff = (encoded - self.last_encoded) & 0x03
        
        if diff == 0x03 or diff == 0x01:
            self.value -= 1
        elif diff == 0x02 or diff == 0x00:
            self.value += 1
            
        self.last_encoded = encoded

    def get_count(self):
        return self.value

    def reset(self):
        self.value = 0

