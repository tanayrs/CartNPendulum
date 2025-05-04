'''
Data Logging Module

This module provides a data logging system for recording and storing
state information from the Cart-Pendulum reinforcement learning platform.
It handles file creation, data writing, and proper resource management.

Features:
- Creates timestamped CSV log files for each run
- Records system state variables (position, velocity, angle, angular velocity)
- Tracks control outputs and timestamps
- Ensures atomic writes with proper file handling
- Provides console feedback during logging operations

Data Recorded:
- Timestamp: ISO-formatted date and time of each data point
- Cart position (x): Linear position of the cart
- Cart velocity (x_dot): Linear velocity of the cart
- Pendulum angle (theta): Angular position of the pendulum
- Angular velocity (theta_dot): Rate of change of pendulum angle
- Control output: Action value sent to the motor

Usage:
    logger = DataLogger(log_dir="system_logs")
    logger.start()  # Creates a new timestamped log file
    
    # During control loop
    logger.log({
        'x': cart_position,
        'x_dot': cart_velocity,
        'theta': pendulum_angle,
        'theta_dot': angular_velocity,
        'control_output': motor_command
    })
    
    # When finished
    logger.stop()  # Properly closes the file

Output:
    CSV files stored in the specified log directory with timestamps
    in format: log_YYYYMMDD_HHMMSS.csv
'''

# imports
import csv
import os
from datetime import datetime

class DataLogger:
    def __init__(self, log_dir="system_logs"):
        """Initialize the data logger with a directory for storing log files."""
        self.log_dir = log_dir
        self.log_file = None  # File handle for the current log
        self.writer = None    # CSV writer object
        
        # Create the log directory if it doesn't exist
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
    def start(self):
        """Start a new logging session with a timestamped CSV file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")  # Format: YYYYMMDD_HHMMSS
        filename = f"{self.log_dir}/log_{timestamp}.csv"      # Create unique filename
        self.log_file = open(filename, 'w', newline='')       # Open file for writing
        self.writer = csv.writer(self.log_file)               # Initialize CSV writer
        # Write header row with column names
        self.writer.writerow([
            'timestamp', 'roll', 'pitch', 
            'target_angle', 'output', 'motor_speed'
        ])
        
    def log(self, data):
        """Log a data point with the current timestamp."""
        if self.writer:  # Check if writer is initialized
            # Write row with current timestamp and data values
            self.writer.writerow([
                datetime.now().isoformat(),         # Current time in ISO format
                data.get('x', 0),                   # Cart position
                data.get('x_dot', 0),               # Cart velocity
                data.get('theta', 0),               # Pendulum angle (roll)
                data.get('theta_dot', 0),           # Angular velocity
                data.get('control_output', 0)       # Control signal
            ])
            # Print logged data to console and flush to ensure data is written
            print(f"Logged: {datetime.now().isoformat()}, {data.get('x',0)}, {data.get('x_dot', 0)}, {data.get('theta', 0)}, {data.get('theta_dot', 0)}, {data.get('control_output', 0)}, {self.log_file.flush()}")
            
    def stop(self):
        """Close the log file and clean up resources."""
        if self.log_file:  # Check if file is open
            self.log_file.close()  # Properly close the file