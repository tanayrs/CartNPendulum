# Datalogger for main deployment, saves trace of episodes

import csv
import os
from datetime import datetime

class DataLogger:
    def __init__(self, log_dir="system_logs"):
        self.log_dir = log_dir
        self.log_file = None
        self.writer = None
        
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
    def start(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.log_dir}/log_{timestamp}.csv"
        self.log_file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.log_file)
        self.writer.writerow([
            'timestamp', 'roll', 'pitch', 
            'target_angle', 'output', 'motor_speed'
        ])
        
    def log(self, data):
        if self.writer:
            self.writer.writerow([
                datetime.now().isoformat(),
                data.get('x', 0),
                data.get('x_dot', 0),
                data.get('theta', 0), # roll angle
                data.get('theta_dot', 0), # gyro rate x
                data.get('control_output', 0)
            ])
            print(f"Logged: {datetime.now().isoformat()}, {data.get('x',0)}, {data.get('x_dot', 0)}, {data.get('theta', 0)}, {data.get('theta_dot', 0)}, {data.get('control_output', 0)}, {self.log_file.flush()}")
            
    def stop(self):
        if self.log_file:
            self.log_file.close()

