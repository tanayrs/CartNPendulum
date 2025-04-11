import os
import sys
import warnings
import gymnasium as gym
from stable_baselines3 import DQN
from stable_baselines3.common.evaluation import evaluate_policy
from gymnasium.envs.registration import register

warnings.filterwarnings("ignore")

# Determine the project root (the folder that contains both 'env' and 'models')
try:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
except NameError:
    project_root = os.getcwd()

if project_root not in sys.path:
    sys.path.append(project_root)

# Import the custom environment module (from the 'env' folder)
import env.custom_cartpole

# Register the custom environment so that gymnasium can find it.
try:
    register(
        id="CustomCartPole-v1",
        entry_point="env.custom_cartpole:CustomCartPoleEnv",  # ensure this matches your class name
    )
except gym.error.Error as e:
    if "already registered" in str(e):
        pass
    else:
        raise e

# Create the custom environment
environment_name = "CustomCartPole-v1"
env = gym.make(environment_name)

# Prepare training logging directory
log_path = os.path.join("Training", "Logs")
os.makedirs(log_path, exist_ok=True)

# Instantiate the DQN model (using a non-vectorized env for DQN)
model = DQN('MlpPolicy', env, verbose=1, tensorboard_log=log_path)

# Train the model (remove callback if undefined)
model.learn(total_timesteps=20000)

# Save the trained model
dqn_path = os.path.join('Training', 'Saved Models', 'DQN_model')
os.makedirs(os.path.dirname(dqn_path), exist_ok=True)
model.save(dqn_path)

# Reload the model (using the same environment)
model = DQN.load(dqn_path, env=env)

# Evaluate the policy over 10 episodes with rendering enabled (if your setup supports rendering)
evaluate_policy(model, env, n_eval_episodes=10, render=True)

env.close()
