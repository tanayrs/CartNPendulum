import threading
import time
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
                print("Safety cutoff triggered!")
            time.sleep(0.01)
            
    def stop(self):
        self.running = False

