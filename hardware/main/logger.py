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
                data.get('roll', 0),
                data.get('pitch', 0),
                data.get('target', 0),
                data.get('output', 0),
                data.get('speed', 0)
            ])
            print(f"Logged: {datetime.now().isoformat()}, {data.get('roll',0)}, {data.get('pitch', 0)}, {data.get('target', 0)}, {data.get('output', 0)}, {data.get('speed', 0)}, {self.log_file.flush()}")
            
    def stop(self):
        if self.log_file:
            self.log_file.close()

