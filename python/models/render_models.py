# This script loads a saved model and renders the environment.

# imports
import os
import sys
import time
import gymnasium as gym
from stable_baselines3 import PPO, DQN
from gymnasium.envs.registration import register
import numpy as np

# --- Adjust sys.path so that the custom environment is accessible ---
try:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
except NameError:
    project_root = os.getcwd()

if project_root not in sys.path:
    sys.path.append(project_root)

# --- Create the environment with render mode enabled ---
# When rendering, "human" mode opens a window to visualize the simulation.
try:
    register(
        id="CustomCartPole-v1",
        entry_point="environment.custom_cartpole:CartPoleEnv",
    )
except gym.error.Error as e:
    # If the environment has already been registered, ignore the error.
    if "already registered" in str(e):
        pass
    else:
        raise e

# Create an instance of your custom environment using gym.make
environment_name = "CustomCartPole-v1"
env = gym.make(environment_name, render_mode="human")

# --- Load your saved models ---
# Adjust these paths to where you have saved your models.
# ppo_model_path = os.path.join("Training", "Saved Models", "PPO_model")
dqn_model_path = os.path.join("Training", "Saved Models", "DQN_model")

# Uncomment the model you want to test, or test both in succession:

# Load PPO model:
# model = PPO.load(ppo_model_path, env=env)
# print("Loaded PPO model.")

# Alternatively, load DQN model:
model = DQN.load(dqn_model_path, env=env)
print("Loaded DQN model.")

# --- Render the model in a few episodes ---
episodes = 150
for episode in range(1, episodes + 1):
    obs, _ = env.reset()
    terminated = False
    truncated = False
    score = 0
    steps = 0
    x0, x_dot0, theta0, theta_dot0 = obs
    print(f"Starting episode {episode}...")
    print(f"Initial theta: {np.degrees(theta0):.2f} deg, Initial x: {x0:.2f} m")
    while not terminated and not truncated:
        # call env.render() if needed
        env.render()
        # Get action from the model (deterministic=True for consistent behavior during evaluation)
        action, _ = model.predict(obs, deterministic=True)
        # Step the environment
        obs, reward, terminated, truncated, info = env.step(action)
        score += reward
        steps += 1
        # Slow down the simulation for visualization (optional)
        time.sleep(0.001)
    print(f"Episode {episode} finished")
    print(f"Score = {score:.2f}, Episode length = {steps} steps")

env.close()