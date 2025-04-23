import os
import sys
import warnings
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from gymnasium.envs.registration import register
#import env.custom_cartpole


#warnings.filterwarnings("ignore")

# ✅ Determine the project root folder (which contains both env and models)
try:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) 
except NameError:
    project_root = os.getcwd()

if project_root not in sys.path:
    sys.path.append(project_root)

# ✅ Import your custom environment module from the env folder
import env.custom_cartpole1

# ✅ Register your custom environment so gymnasium can find it.
# Make sure the entry point string matches the module path and your class name.
try:
    register(
        id="CustomCartPole-v1",
        entry_point="env.custom_cartpole1:CartPoleEnv1",
    )
except gym.error.Error as e:
    # If the environment has already been registered, ignore the error.
    if "already registered" in str(e):
        pass
    else:
        raise e

# ✅ Create an instance of your custom environment using gym.make
environment_name = "CustomCartPole-v1"
env_instance = gym.make(environment_name)

# ✅ (Optional) Run a few episodes for visual inspection (rendering disabled on headless environments)
episodes = 5
for episode in range(1, episodes + 1):
    state, _ = env_instance.reset()
    done = False
    score = 0
    while not done:
        # Uncomment the following line if rendering is supported
        # env_instance.render()
        action = env_instance.action_space.sample()
        n_state, reward, done, _, _ = env_instance.step(action)
        score += reward
    print(f"Episode: {episode} Score: {score}")

env_instance.close()

# ✅ Prepare training logging directory
log_path = os.path.join("Training", "Logs")
os.makedirs(log_path, exist_ok=True)

# Wrap the environment in a DummyVecEnv for stable-baselines3
vec_env = DummyVecEnv([lambda: gym.make(environment_name)])

# ✅ Train PPO with the custom environment
model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log=log_path)
model.learn(total_timesteps=100)

# ✅ Save the trained model
PPO_path = os.path.join("Training", "Saved Models", "PPO_model")
os.makedirs(os.path.dirname(PPO_path), exist_ok=True)
model.save(PPO_path)

print("Model saved at:", PPO_path)
