# PPO (Proximal Policy Optimization) training script for a custom CartPole environment

# imports
import os
import sys
import warnings
import gymnasium as gym  # Modern version of OpenAI Gym for RL environments
from stable_baselines3 import PPO  # PPO algorithm implementation
from stable_baselines3.common.vec_env import DummyVecEnv  # For vectorized environments
from stable_baselines3.common.callbacks import BaseCallback  # Add this import
from gymnasium.envs.registration import register  # For registering custom environments
import matplotlib.pyplot as plt  # For plotting training results

#warnings.filterwarnings("ignore")

# Custom callback for loss convergence
class PPOLossConvergenceCallback(BaseCallback):
    def __init__(self, verbose=1, threshold=1e-5, check_freq=100):
        super(PPOLossConvergenceCallback, self).__init__(verbose)
        self.threshold = threshold
        self.check_freq = check_freq
        # Track losses
        self.policy_losses = []
        self.value_losses = []
        self.entropy_values = []
        self.iterations = []
        self.rewards = []
        
    def _on_step(self):
        # Track episode rewards
        if len(self.model.ep_info_buffer) > 0 and len(self.model.ep_info_buffer[-1]) > 0:
            self.rewards.append(self.model.ep_info_buffer[-1]['r'])
            
        # Extract losses from logger
        if hasattr(self.model, 'logger') and len(self.model.logger.name_to_value) > 0:
            policy_loss = None
            value_loss = None
            entropy = None
            
            for key, value in self.model.logger.name_to_value.items():
                if 'policy_loss' in key:
                    policy_loss = value
                elif 'value_loss' in key:
                    value_loss = value
                elif 'entropy' in key or 'policy_entropy' in key:
                    entropy = value
            
            # Only add losses if we found them
            if policy_loss is not None:
                self.iterations.append(self.n_calls)
                self.policy_losses.append(policy_loss)
                if value_loss is not None:
                    self.value_losses.append(value_loss)
                if entropy is not None:
                    self.entropy_values.append(entropy)
                
                # Check for convergence every check_freq steps
                if self.n_calls % self.check_freq == 0:
                    if self.verbose > 0:
                        print(f"Current policy loss: {policy_loss:.8f}, threshold: {self.threshold}")
                    
                    # Stop if policy loss is below threshold
                    if abs(policy_loss) < self.threshold:
                        print(f"Convergence reached: policy loss {policy_loss:.8f} < {self.threshold}")
                        return False  # Stop training
        
        return True  # Continue training


# Determine the project root folder (which contains both env and models)
try:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))  # Get parent directory of current file
except NameError:
    project_root = os.getcwd()  # Fallback to current working directory if running interactively

if project_root not in sys.path:
    sys.path.append(project_root)  # Add project root to Python path for imports


# Register your custom environment so gymnasium can find it.
# Make sure the entry point string matches the module path and your class name.
try:
    register(
        id="CustomCartPole-v1",  # ID to reference this environment
        entry_point="environment.custom_cartpole:CartPoleEnv",  # Path to environment class
    )
except gym.error.Error as e:
    # If the environment has already been registered, ignore the error.
    if "already registered" in str(e):
        pass  # Skip if environment is already registered
    else:
        raise e  # Re-raise other errors

# Create an instance of your custom environment using gym.make
environment_name = "CustomCartPole-v1"  # The registered ID
env_instance = gym.make(environment_name)  # Create environment instance

# Prepare training logging directory
log_path = os.path.join("Training", "Logs")  # Path for tensorboard logs
os.makedirs(log_path, exist_ok=True)  # Create directory if it doesn't exist

# Wrap the environment in a DummyVecEnv for stable-baselines3
vec_env = DummyVecEnv([lambda: gym.make(environment_name)])  # SB3 requires vectorized environments

# Initialize the loss convergence callback
loss_callback = PPOLossConvergenceCallback(threshold=1e-5)

# Train PPO with the custom environment - note the increased max timesteps
model = PPO(
    "MlpPolicy", 
    vec_env, 
    verbose=1, 
    tensorboard_log=log_path,
    # You can optionally tune these parameters for faster convergence
    learning_rate=0.0003,
    n_steps=2048,
    batch_size=64,
    n_epochs=10
)

# Train until convergence or max timesteps
print("Starting training until policy loss < 1e-5...")
model.learn(total_timesteps=1000000, callback=loss_callback)  # Increased max timesteps

# Save the trained model
PPO_path = os.path.join("Training", "Saved Models", "PPO_model")
os.makedirs(os.path.dirname(PPO_path), exist_ok=True)
model.save(PPO_path)
print("Model saved at:", PPO_path)

# Plot losses and rewards
plt.figure(figsize=(15, 10))

# Plot policy loss
plt.subplot(2, 2, 1)
plt.plot(loss_callback.iterations, loss_callback.policy_losses)
plt.axhline(y=1e-5, color='r', linestyle='--', label='Threshold (1e-5)')
plt.title('Policy Loss Convergence')
plt.xlabel('Training Steps')
plt.ylabel('Policy Loss')
plt.legend()
plt.yscale('log')  # Log scale helps visualize small values

# Plot value loss
if len(loss_callback.value_losses) > 0:
    plt.subplot(2, 2, 2)
    plt.plot(loss_callback.iterations, loss_callback.value_losses)
    plt.title('Value Function Loss')
    plt.xlabel('Training Steps')
    plt.ylabel('Value Loss')

# Plot entropy
if len(loss_callback.entropy_values) > 0:
    plt.subplot(2, 2, 3)
    plt.plot(loss_callback.iterations, loss_callback.entropy_values)
    plt.title('Policy Entropy')
    plt.xlabel('Training Steps')
    plt.ylabel('Entropy')

# Plot rewards
if len(loss_callback.rewards) > 0:
    plt.subplot(2, 2, 4)
    plt.plot(loss_callback.rewards)
    plt.title('Episode Rewards')
    plt.xlabel('Episode')
    plt.ylabel('Reward')

plt.tight_layout()

# Save the plot
plot_path = os.path.join("Training", "Plots", "ppo_convergence.png")
os.makedirs(os.path.dirname(plot_path), exist_ok=True)
plt.savefig(plot_path)
print(f"Convergence plot saved at: {plot_path}")
plt.show()