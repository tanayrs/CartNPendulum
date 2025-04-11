import os
import sys
import time
import gymnasium as gym
from stable_baselines3 import PPO, DQN

# --- Adjust sys.path so that the custom environment is accessible ---
try:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
except NameError:
    project_root = os.getcwd()

if project_root not in sys.path:
    sys.path.append(project_root)

# --- Create the environment with render mode enabled ---
# When rendering, "human" mode opens a window to visualize the simulation.
env = gym.make("CartPole-v1", render_mode="human")

# --- Load your saved models ---
# Adjust these paths to where you have saved your models.
ppo_model_path = os.path.join("Training", "Saved Models", "PPO_model")
dqn_model_path = os.path.join("Training", "Saved Models", "DQN_model")

# Uncomment the model you want to test, or test both in succession:

# Load PPO model:
#model = PPO.load(ppo_model_path, env=env)
#print("Loaded PPO model.")

# Alternatively, load DQN model:
model = DQN.load(dqn_model_path, env=env)
print("Loaded DQN model.")

# --- Render the model in a few episodes ---
episodes = 150
for episode in range(1, episodes + 1):
    obs, _ = env.reset()
    done = False
    score = 0
    print(f"Starting episode {episode}...")
    while not done:
        # Optional: call env.render() if needed
        # (Note: some Gymnasium environments automatically update the display in 'human' mode)
        env.render()
        # Get action from the model (deterministic=True for consistent behavior during evaluation)
        action, _ = model.predict(obs, deterministic=True)
        # Step the environment
        obs, reward, done, truncated, info = env.step(action)
        score += reward
        # Slow down the simulation for visualization (optional)
        time.sleep(0.02)
    print(f"Episode {episode}: Score = {score}")

env.close()
