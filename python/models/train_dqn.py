# DQN (Deep Q-Network) training script for a custom CartPole environment

#imports
import os
import sys
import warnings
import gymnasium as gym  # Modern version of OpenAI Gym for RL environments
from stable_baselines3 import DQN  # Implementation of Deep Q-Learning algorithm
from stable_baselines3.common.evaluation import evaluate_policy  # For evaluating trained models
from gymnasium.envs.registration import register  # For registering custom environments

#warnings.filterwarnings("ignore")  # Commented out warning suppression

# Determine the project root (the folder that contains both 'env' and 'models')
try:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))  # Get parent directory
except NameError:
    project_root = os.getcwd()  # Fallback to current working directory if __file__ is not defined

if project_root not in sys.path:
    sys.path.append(project_root)  # Add project root to system path for imports


# Register the custom environment so that gymnasium can find it.
try:
    register(
        id="CustomCartPole-v1",  # ID used to refer to this environment
        entry_point="environment.custom_cartpole:CartPoleEnv",  # Path to the environment class
    )
except gym.error.Error as e:
    if "already registered" in str(e):
        pass  # Skip if environment is already registered
    else:
        raise e  # Re-raise if it's a different error

# Create the custom environment
environment_name = "CustomCartPole-v1"
environment = gym.make(environment_name)  # Instantiate the environment

# Prepare training logging directory
log_path = os.path.join("Training", "Logs")  # Path for tensorboard logs
os.makedirs(log_path, exist_ok=True)  # Create directory if it doesn't exist

# Instantiate the DQN model (using a non-vectorized env for DQN)
model = DQN('MlpPolicy', environment, verbose=1, tensorboard_log=log_path)  # MlpPolicy uses a neural network

# Train the model (remove callback if undefined)
model.learn(total_timesteps=200000)  # Train for 200K steps

# Save the trained model
dqn_path = os.path.join('Training', 'Saved Models', 'DQN_model')  # Path to save model
os.makedirs(os.path.dirname(dqn_path), exist_ok=True)  # Create directory if it doesn't exist
model.save(dqn_path)  # Save model to disk

# Reload the model (using the same environment)
# model = DQN.load(dqn_path, env=environment)  # Load model from disk

# Evaluate the policy over 10 episodes with rendering enabled (if your setup supports rendering)
# evaluate_policy(model, environment, n_eval_episodes=10, render=True)  # Run 10 test episodes

environment.close()  # Clean up environment resources