# PPO (Proximal Policy Optimization) training script for a custom CartPole environment

# imports
import os
import sys
import warnings
import gymnasium as gym  # Modern version of OpenAI Gym for RL environments
from stable_baselines3 import PPO  # PPO algorithm implementation
from stable_baselines3.common.vec_env import DummyVecEnv  # For vectorized environments
from gymnasium.envs.registration import register  # For registering custom environments

#warnings.filterwarnings("ignore")

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

# Train PPO with the custom environment
model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log=log_path)  # Initialize PPO with MLP policy
model.learn(total_timesteps=200000)  # Train for 200k steps

# Save the trained model
PPO_path = os.path.join("Training", "Saved Models", "PPO_model")  # Path to save model
os.makedirs(os.path.dirname(PPO_path), exist_ok=True)  # Ensure directory exists
model.save(PPO_path)  # Save trained model to disk

print("Model saved at:", PPO_path)  # Print confirmation message with save location