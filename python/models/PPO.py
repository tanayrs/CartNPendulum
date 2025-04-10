# attempt 1, unverified

import os
import sys
import warnings
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy

warnings.filterwarnings("ignore")

# Add path to access custom env
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Import custom env (this registers it)
import env.custom_cartpole

# Use custom environment
environment_name = "CustomCartPole-v1"
env = gym.make(environment_name)

# Visual inspection
episodes = 5
for episode in range(1, episodes + 1):
    state, _ = env.reset()
    done = False
    score = 0

    while not done:
        # env.render()  # Comment out in Kaggle — doesn't work
        action = env.action_space.sample()
        n_state, reward, done, _, _ = env.step(action)
        score += reward

    print(f"Episode:{episode} Score:{score}")

env.close()

# Prepare for training
log_path = os.path.join("Training", "Logs")
os.makedirs(log_path, exist_ok=True)

# Wrap in DummyVecEnv
env = DummyVecEnv([lambda: gym.make(environment_name)])

# Train PPO
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=20000)

# Save model
PPO_path = os.path.join("Training", "Saved Models", "PPO_model")
os.makedirs(os.path.dirname(PPO_path), exist_ok=True)
model.save(PPO_path)

print("Model saved at:", PPO_path)

