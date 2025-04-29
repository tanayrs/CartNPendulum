# File to test different filters for encoder, when we measure xdot, the numerical derivative tends to be noisy so we need to add a filter
# Now to find the best filter, we need to compare them, thats what this file does.

'''
Encoder Velocity Filter Comparison

This script analyzes and compares different moving average filtering techniques
for smoothing encoder velocity signals

Problem:
When calculating velocity from position encoder data through numerical 
differentiation, the results are often noisy. This noise can negatively impact 
control performance, especially in precision applications like balancing a pendulum.

Features:
- Processes raw encoder tick data from CSV files
- Calculates angular position from encoder ticks
- Computes angular velocity through time differentiation
- Adds controlled noise to demonstrate filter effectiveness
- Applies moving average filters with various window sizes (2, 4, 6, 8)
- Generates comparative visualizations of filter performance
- Exports processed data for further analysis

Purpose:
This analysis helps determine the optimal window size for velocity filtering
in the encoder processing pipeline. The goal is to balance noise reduction with
minimizing phase lag, which is critical for real-time control applications.

Visualization:
- Individual plots for each filter window size
- Composite plot showing all filters together for direct comparison
- Side-by-side visualization of noise reduction vs. responsiveness
    
Output:
    - Multi-panel visualization of filter performance
    - Filtered data saved to 'data/filtered_velocity_data.csv'
    - Console output with data summary
'''

# imports
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Sample CSV data path
csv_file = './data/deadband_test_2025-04-19 12:37:43.183066.csv' 

# Parse the CSV data into a pandas DataFrame
df = pd.read_csv(csv_file)

# Parameters for the encoder
ppr = 2262                  # Pulses per revolution of the encoder
ticks_per_rev = 4 * ppr     # Total ticks per revolution (quadrature encoding)

# Convert raw encoder ticks to angular position in degrees
df['position'] = (df['ticks'] / ticks_per_rev) * 360

# Calculate velocity by taking derivatives
df['time_diff'] = df['time'].diff()                                # Time between samples
df['position_diff'] = df['position'].diff()                        # Change in position
df['calculated_velocity'] = df['position_diff'] / df['time_diff']  # Angular velocity (deg/s)

# Handle the first row which has NaN due to the diff operation
df.loc[0, 'calculated_velocity'] = df.loc[1, 'calculated_velocity'] if len(df) > 1 else 0

# Add synthetic noise to velocity to demonstrate filtering effectiveness
np.random.seed(42)  # Set seed for reproducible noise
noise = np.random.normal(0, 0.1, size=len(df))  # Generate Gaussian noise
df['noisy_velocity'] = df['calculated_velocity'] + noise  # Add noise to velocity

# Apply moving average filters with different window sizes
window_sizes = [2, 4, 6, 8]  # Different filter window sizes to compare
for window_size in window_sizes:
    column_name = f'window_{window_size}'
    # Apply rolling window average (each point becomes average of itself and previous points)
    df[column_name] = df['noisy_velocity'].rolling(window=window_size, min_periods=1).mean()

# Create visualization plots
plt.figure(figsize=(15, 18))  # Create large figure
gs = GridSpec(6, 1, figure=plt.gcf())  # 6 rows, 1 column grid

# Plot 1: Show the original noisy velocity data
ax1 = plt.subplot(gs[0, 0])
ax1.plot(df.index, df['noisy_velocity'], label='Noisy Velocity', color='blue')
ax1.set_title('Original Noisy Velocity')
ax1.set_ylabel('Velocity (deg/s)')
ax1.grid(True)
ax1.legend()

# Plots 2-5: Individual comparisons of each filter window size
for i, window_size in enumerate(window_sizes):
    column_name = f'window_{window_size}'
    ax = plt.subplot(gs[i+1, 0])
    ax.plot(df.index, df[column_name], label=f'Window Size {window_size}', 
            color=['green', 'orange', 'red', 'purple'][i])
    ax.set_title(f'Original vs. Window Size {window_size}')
    ax.set_ylabel('Velocity (deg/s)')
    ax.grid(True)
    ax.legend()

# Plot 6: Compare all filtered signals together
ax_all = plt.subplot(gs[5, 0])
ax_all.plot(df.index, df['noisy_velocity'], label='Original', color='blue')
for i, window_size in enumerate(window_sizes):
    column_name = f'window_{window_size}'
    ax_all.plot(df.index, df[column_name], label=f'Window Size {window_size}')
ax_all.set_title('All Filtered Velocities Comparison')
ax_all.set_xlabel('Data Point Index')
ax_all.set_ylabel('Velocity (deg/s)')
ax_all.grid(True)
ax_all.legend()

# Ensure good spacing between subplots
plt.tight_layout()
plt.show()

# Print the first 10 rows to verify filter calculations
print(df[['time', 'ticks', 'position', 'noisy_velocity'] + 
       [f'window_{size}' for size in window_sizes]].head(10))

# Save the processed data with all filtered versions
df.to_csv('data/filtered_velocity_data.csv', index=True)
