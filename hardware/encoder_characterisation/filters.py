# compare filters of encoder time derivative, explained under motor deadband compensation

'''
Encoder Velocity Filtering Analysis Script

This script analyzes and compares different moving average filtering techniques for smoothing encoder velocity signals 

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
    - Filtered data saved to 'filtered_velocity_data.csv'
    - Console output with data summary
'''

# imports
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Sample CSV data (copy your actual data here)
csv_file = './data/deadband_test_2025-04-19 12:37:43.183066.csv' 

# Parse the CSV data
df = pd.read_csv(csv_file)

# Parameters
ppr = 2262
ticks_per_rev = 4 * ppr

# Calculate position from ticks
df['position'] = (df['ticks'] / ticks_per_rev) * 360

# Calculate velocity (degrees per second)
df['time_diff'] = df['time'].diff()
df['position_diff'] = df['position'].diff()
df['calculated_velocity'] = df['position_diff'] / df['time_diff']

# Fill the first row velocity (which is NaN due to diff operation)
df.loc[0, 'calculated_velocity'] = df.loc[1, 'calculated_velocity'] if len(df) > 1 else 0

# Add some noise to velocity to demonstrate the filtering effect
np.random.seed(42)  # For reproducibility
noise = np.random.normal(0, 0.1, size=len(df))
df['noisy_velocity'] = df['calculated_velocity'] + noise

# Apply moving average with different window sizes
window_sizes = [2, 4, 6, 8]
for window_size in window_sizes:
    column_name = f'window_{window_size}'
    df[column_name] = df['noisy_velocity'].rolling(window=window_size, min_periods=1).mean()

# Plotting
plt.figure(figsize=(15, 18))
gs = GridSpec(6, 1, figure=plt.gcf())

# Plot 1: Original noisy velocity
ax1 = plt.subplot(gs[0, 0])
ax1.plot(df.index, df['noisy_velocity'], label='Noisy Velocity', color='blue')
ax1.set_title('Original Noisy Velocity')
ax1.set_ylabel('Velocity (deg/s)')
ax1.grid(True)
ax1.legend()

# Plots 2-5: Original vs each window size
for i, window_size in enumerate(window_sizes):
    column_name = f'window_{window_size}'
    ax = plt.subplot(gs[i+1, 0])
    ax.plot(df.index, df[column_name], label=f'Window Size {window_size}', 
            color=['green', 'orange', 'red', 'purple'][i])
    ax.set_title(f'Original vs. Window Size {window_size}')
    ax.set_ylabel('Velocity (deg/s)')
    ax.grid(True)
    ax.legend()

# Plot 6: All together
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

plt.tight_layout()
plt.show()

# Print data summary to verify calculations
print(df[['time', 'ticks', 'position', 'noisy_velocity'] + 
       [f'window_{size}' for size in window_sizes]].head(10))

# Save the filtered data to CSV if needed
df.to_csv('data/filtered_velocity_data.csv', index=True)
