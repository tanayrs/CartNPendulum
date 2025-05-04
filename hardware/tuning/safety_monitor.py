'''
Safety monitoring system

This module implements a safety monitor that runs in a separate thread to
continuously check the state of the cart-pendulum environment. If the cart
position or pendulum angle exceeds specified safety thresholds, the motor
is automatically stopped to prevent damage to the hardware.

it runs as a daemon thread that can be started and stopped
alongside the main control program.
'''

# imports
import threading
#import time
from motor import HardwarePWMMotor

class SafetyMonitor(threading.Thread):
    def __init__(self, env, threshold=0.3):
        super().__init__()
        self.env = env
        self.threshold = threshold
        self.running = True
        self.daemon = True

    def run(self):
        while self.running:
            obs = self.env._get_obs()
            if abs(obs[0]) > self.threshold or abs(obs[2]) > 0.2095:
                self.env.motor.stop()
                print(f"Safety cutoff triggered! x = {obs[0]}, theta = {obs[2]}")

    def stop(self):
        self.running = False