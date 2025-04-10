# attempt 2, unverified

import os
import sys
import warnings
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from gymnasium.envs.registration import register

warnings.filterwarnings("ignore")

# Fix for interactive environments: if __file__ is undefined, use current working directory.
try:
    current_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
except NameError:
    current_dir = os.getcwd()

# Add path to access custom env
sys.path.append(current_dir)

# Import custom env (this imports your env module)
import env.custom_cartpole

# Register your custom environment if it hasn't been already.
# This allows gym.make("CustomCartPole-v1") to find your environment.
try:
    register(
        id="CustomCartPole-v1",
        entry_point="env.custom_cartpole:CustomCartpoleEnv",  # be sure the class name matches exactly
    )
except gym.error.Error as e:
    # If the environment is already registered, we catch the error and continue
    if "already registered" in str(e):
        pass
    else:
        raise e

# Use your custom environment
environment_name = "CustomCartPole-v1"
env_instance = gym.make(environment_name)

# Visual inspection — not rendered in Kaggle
episodes = 5
for episode in range(1, episodes + 1):
    state, _ = env_instance.reset()
    done = False
    score = 0

    while not done:
        # env_instance.render()  # ⚠ Comment out in Kaggle — doesn't work
        action = env_instance.action_space.sample()
        n_state, reward, done, _, _ = env_instance.step(action)
        score += reward

    print(f"Episode:{episode} Score:{score}")

env_instance.close()

# Prepare for training: create log directory
log_path = os.path.join("Training", "Logs")
os.makedirs(log_path, exist_ok=True)

# Wrap in DummyVecEnv
vec_env = DummyVecEnv([lambda: gym.make(environment_name)])

# Train PPO
model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=20000)

# Save model
PPO_path = os.path.join("Training", "Saved Models", "PPO_model")
os.makedirs(os.path.dirname(PPO_path), exist_ok=True)
model.save(PPO_path)

print("Model saved at:", PPO_path)
