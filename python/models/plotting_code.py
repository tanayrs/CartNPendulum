# code to plot PPO training convergence metrics from TensorBoard logs

# imports
import os
import numpy as np
import matplotlib.pyplot as plt
from tensorflow.python.summary.summary_iterator import summary_iterator
from collections import defaultdict

# Path to tensorboard log file
log_file = "Training/Logs/PPO_5/events.out.tfevents.1745937462.1D-ta.17299.0"

# Extract data from tensorboard file
data = defaultdict(list)
steps = defaultdict(list)

print(f"Reading TensorBoard log file: {log_file}")

# First just identify all available metrics
available_metrics = set()
try:
    for event in summary_iterator(log_file):
        for value in event.summary.value:
            available_metrics.add(value.tag)
    
    print("Available metrics in the log file:")
    for metric in sorted(available_metrics):
        print(f"  - {metric}")
    
    # Now extract the data
    for event in summary_iterator(log_file):
        for value in event.summary.value:
            if event.step > 0:  # Skip initial step
                steps[value.tag].append(event.step)
                data[value.tag].append(value.simple_value)
except Exception as e:
    print(f"Error reading log file: {e}")

# Function for smoothing noisy data
def smooth(y, box_pts=10):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

# Check if any data was found
if not data:
    print("No data found in the log file.")
    exit(1)

# Define metrics we want to plot with priority based on the output
# Include all metrics available in the terminal output
metrics_to_plot = [
    # Primary convergence metrics - these are most important
    ('train/policy_gradient_loss', 'Policy Gradient Loss', 'log', 1e-5),
    ('train/value_loss', 'Value Loss', 'log', None),
    ('train/loss', 'Total Loss', 'log', None),
    
    # Secondary analysis metrics
    ('train/entropy_loss', 'Policy Entropy', 'linear', None),
    ('train/approx_kl', 'KL Divergence', 'log', None),
    ('train/clip_fraction', 'Clip Fraction', 'linear', None),
    
    # Additional metrics if available
    ('train/explained_variance', 'Explained Variance', 'linear', None),
    ('train/learning_rate', 'Learning Rate', 'log', None),
    ('time/fps', 'Training Speed (FPS)', 'linear', None)
]

# Create plots - always use a 3x3 grid to accommodate all possible metrics
fig, axs = plt.subplots(3, 3, figsize=(18, 14))
axs = axs.flatten()  # Make it a 1D array for easy indexing

# Track which metrics we successfully plotted
plotted_metrics = []

# Plot each metric we want to include
for i, (metric_name, title, scale, threshold) in enumerate(metrics_to_plot):
    if i >= len(axs):
        print(f"Warning: More metrics than plot spaces. Skipping {metric_name}")
        break
        
    # Skip metrics not in the data
    if metric_name not in data:
        continue
        
    plotted_metrics.append(metric_name)
    ax = axs[i]
    x = np.array(steps[metric_name]) / 1000  # Convert to thousands of steps
    y = np.array(data[metric_name])
    
    # Determine color based on metric type
    if 'policy' in metric_name:
        color = 'blue'
    elif 'value' in metric_name:
        color = 'green'
    elif 'entropy' in metric_name:
        color = 'purple'
    elif 'kl' in metric_name:
        color = 'orange'
    else:
        color = 'gray'
    
    # Plot raw data with alpha
    ax.plot(x, y, alpha=0.3, color=color, label='Raw')
    
    # Plot smoothed data - adjust box_pts based on data density
    smooth_pts = min(len(y) // 20 + 1, 50)  # Adaptive smoothing window
    ax.plot(x, smooth(y, box_pts=smooth_pts), linewidth=2, color=color, label='Smoothed')
    
    # Add threshold line if applicable
    if threshold is not None:
        ax.axhline(y=threshold, color='r', linestyle='--', 
                  label=f"Threshold ({threshold})")
        
        # Check if threshold was reached
        if np.min(y) < threshold:
            min_idx = np.argmin(y)
            min_val = y[min_idx]
            convergence_step = x[min_idx]
            ax.text(0.05, 0.05, f"Threshold reached: {min_val:.7f}\nStep: {convergence_step*1000:.0f}",
                   transform=ax.transAxes, fontsize=10, 
                   bbox=dict(facecolor='green', alpha=0.2))
        else:
            ax.text(0.05, 0.05, "Threshold not reached",
                   transform=ax.transAxes, fontsize=10, 
                   bbox=dict(facecolor='red', alpha=0.2))
    
    # Add min value annotation
    min_idx = np.argmin(y)
    min_x, min_y = x[min_idx], y[min_idx]
    ax.scatter([min_x], [min_y], color='red', s=40, zorder=5)
    ax.annotate(f'Min: {min_y:.6f}', 
               xy=(min_x, min_y), 
               xytext=(min_x, min_y*1.5 if min_y > 0 else min_y*0.5), 
               fontsize=8,
               arrowprops=dict(facecolor='black', shrink=0.05, width=1))
    
    # Set up the axis
    ax.set_title(title)
    ax.set_xlabel('Training Steps (thousands)')
    ax.set_ylabel(title)
    
    # Set y-scale if appropriate
    if scale == 'log' and np.all(y > 0):
        ax.set_yscale('log')
    
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

# Remove any unused subplots
for i in range(len(plotted_metrics), len(axs)):
    fig.delaxes(axs[i])

plt.suptitle('PPO Training Convergence Analysis', fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Make room for the suptitle

# Save the main plot
plot_dir = "Training/Plots"
os.makedirs(plot_dir, exist_ok=True)
plot_path = os.path.join(plot_dir, "ppo_complete_analysis.png")
plt.savefig(plot_path, dpi=300, bbox_inches='tight')
print(f"Complete analysis plots saved to {plot_path}")

# Create a dedicated summary plot with the most important metrics for convergence
plt.figure(figsize=(16, 8))

# Create a 2x2 layout for the most important metrics
key_metrics = ['train/policy_gradient_loss', 'train/value_loss', 'train/loss', 'train/approx_kl']
titles = ['Policy Gradient Loss', 'Value Loss', 'Total Loss', 'KL Divergence']
colors = ['blue', 'green', 'red', 'orange']
scales = ['log', 'log', 'log', 'log']
thresholds = [1e-5, None, None, None]

for i, (metric, title, color, scale, threshold) in enumerate(zip(key_metrics, titles, colors, scales, thresholds)):
    if metric not in data:
        continue
        
    plt.subplot(2, 2, i+1)
    x = np.array(steps[metric]) / 1000
    y = np.array(data[metric])
    
    # Plot raw and smoothed
    plt.plot(x, y, alpha=0.3, color=color, label='Raw')
    plt.plot(x, smooth(y, box_pts=min(len(y) // 20 + 1, 50)), 
            linewidth=2, color=color, label='Smoothed')
    
    # Add threshold if applicable
    if threshold is not None:
        plt.axhline(y=threshold, color='r', linestyle='--', 
                  label=f"Threshold ({threshold})")
    
    # Find minimum value for annotation
    min_idx = np.argmin(y)
    min_x, min_y = x[min_idx], y[min_idx]
    
    plt.scatter([min_x], [min_y], color='red', s=60, zorder=5)
    plt.annotate(f'Min: {min_y:.7f}', 
                xy=(min_x, min_y), 
                xytext=(min_x+20, min_y*2 if min_y > 0 else min_y*0.5),
                arrowprops=dict(facecolor='black', shrink=0.05, width=1))
    
    plt.title(title)
    plt.xlabel('Training Steps (thousands)')
    plt.ylabel(title)
    
    if scale == 'log' and np.all(y > 0):
        plt.yscale('log')
        
    plt.grid(True, alpha=0.3)
    plt.legend()

plt.suptitle('PPO Convergence Summary', fontsize=16)
plt.tight_layout(rect=[0, 0, 1, 0.96])

# Save the summary plot
summary_path = os.path.join(plot_dir, "ppo_convergence_summary.png")
plt.savefig(summary_path, dpi=300, bbox_inches='tight')
print(f"Convergence summary saved to {summary_path}")

# Final convergence assessment - did all important metrics converge?
print("\nConvergence Assessment Summary:")
if 'train/policy_gradient_loss' in data:
    min_pg_loss = np.min(data['train/policy_gradient_loss'])
    print(f"Minimum Policy Gradient Loss: {min_pg_loss:.8f}")
    print(f"Threshold: {1e-5:.8f}")
    if min_pg_loss < 1e-5:
        print("✓ Policy gradient loss converged successfully below threshold!")
    else:
        print("✗ Policy gradient loss did not reach the convergence threshold.")
        
if 'train/value_loss' in data:
    min_value_loss = np.min(data['train/value_loss'])
    print(f"Minimum Value Loss: {min_value_loss:.8f}")
    
if 'train/approx_kl' in data:
    mean_kl = np.mean(data['train/approx_kl'][-100:])
    print(f"Mean KL Divergence (last 100 steps): {mean_kl:.8f}")
    print("✓ KL divergence is stable" if mean_kl < 0.02 else "✗ KL divergence is high")

plt.show()