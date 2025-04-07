'''
Data logging for the segway through a UDP socket over WiFi
By: Jia Bhargava, Tanay Srinivasa

Receives data from UDP broadcast, logs it to CSV, and plots it.
'''

import time
import csv
import socket
from matplotlib import pyplot as plt
import pandas as pd
import os
import numpy as np
from datetime import datetime

# File and network configurations
log_dir = './data'
log_file = os.path.join(log_dir, f"pendulum_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
UDP_PORT = 4210  # Must match ESP32's UDP port

def read_from_socket(path, overwrite=False):
    # Ensure file write safety
    if os.path.exists(path) and not overwrite:
        raise FileExistsError("File already exists. Use 'overwrite=True' to replace it.")
    
    # Open CSV and write header
    with open(path, mode='w', newline='') as sensor_file:
        sensor_writer = csv.writer(sensor_file)
        sensor_writer.writerow(['time', 'x', 'theta', 'x_dot', 'theta_dot', 'input', 'Kp', 'Ki', 'Kd', 'Kp_pos', 'Ki_pos', 'Kd_pos'])

    # Create UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.bind(('', UDP_PORT))

    print(f"Listening for UDP packets on port {UDP_PORT}...")

    # Read incoming UDP packets and save to CSV
    while True:
        data, addr = sock.recvfrom(1024)  # Receive UDP packet
        message = data.decode().strip()
        print(f"Received: {message}")

        # Parse CSV-like message (assuming CSV format from ESP32)
        line = message.split(',')
        if len(line) == 12:  # Ensure correct data format
            with open(path, mode='a', newline='') as sensor_file:
                sensor_writer = csv.writer(sensor_file)
                sensor_writer.writerow(line)

def plot_raw(path):
    df = pd.read_csv(path)
    df['time_rel'] = df['time'] - df['time'][0]
    time = df['time_rel']
    x = df['x']
    theta = df['theta']
    x_dot = df['x_dot']
    theta_dot = df['theta_dot']
    input = df['input']
    Kp = df['Kp']
    Ki = df['Ki']
    Kd = df['Kd']
    Kp_pos = df['Kp_pos']
    Ki_pos = df['Ki_pos']
    Kd_pos = df['Kd_pos']

    print(df.describe())

    plt.subplot(3, 2, 1)
    plt.scatter(time, x)
    plt.title('X Position')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (m)')
    

    plt.subplot(3, 2, 2)
    plt.scatter(time, theta)
    plt.title('Theta Angle')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta Angle (deg)')
    
    
    plt.subplot(3, 2, 3)
    plt.scatter(time, x_dot)
    plt.title('X Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('X Velocity (m/s)')
    

    plt.subplot(3, 2, 4)
    plt.scatter(time, theta_dot)
    plt.title('Theta Angular Velocity')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta Angular Velocity (rad/s)')
    

    plt.subplot(3, 2, 5)
    plt.scatter(time, input)
    plt.title('Input PWM')
    plt.xlabel('Time (s)')
    plt.ylabel('PWM')
    

    plt.subplot(3, 2, 6)
    plt.scatter(time, Kp, label='Kp')
    plt.scatter(time, Ki, label='Ki')
    plt.scatter(time, Kd, label='Kd')
    plt.title('PID Gains for tilt')
    plt.xlabel('Time (s)')
    plt.ylabel('Gain Value')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    try:
        read_from_socket(log_file)
    except FileExistsError:
        print('File already exists. Skipping logging.')
    except:
        print('Error occurred during logging. Exiting.')        
    plot_raw(log_file)
