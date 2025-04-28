# File 1: ppo_trainer.py
import os
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
import gymnasium as gym

class SafetyMonitorCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.safety_interventions = 0
        
    def _on_step(self) -> bool:
        if self.locals.get('dones') and self.locals.get('infos')[0].get('safety_triggered'):
            self.safety_interventions += 1
        if self.safety_interventions > 10:
            return False
        return True

class RealWorldPPOTrainer:
    def __init__(self, env_config, policy='MlpPolicy',
                 learning_rate=1e-4, n_steps=512, batch_size=32,
                 n_epochs=10, gamma=0.98, clip_range=0.1):
        
        self.env = RealWorldEnv(**env_config)
        self.model = PPO(
            policy,
            DummyVecEnv([lambda: self.env]),
            learning_rate=learning_rate,
            n_steps=n_steps,
            batch_size=batch_size,
            n_epochs=n_epochs,
            gamma=gamma,
            clip_range=clip_range,
            verbose=1,
            device='cpu'
        )
        self.episode_rewards = []
        self.safety_callback = SafetyMonitorCallback()

    def train(self, total_timesteps):
        self.model.learn(
            total_timesteps=total_timesteps,
            callback=self.safety_callback,
            reset_num_timesteps=False
        )
    
    def save_model(self, path):
        self.model.save(path)
        
    def load_model(self, path):
        self.model = PPO.load(path, env=self.env)

class RealWorldEnv(gym.Env):
    def __init__(self, max_angle=15.0, max_position=0.5, 
                 max_episode_steps=500, safety_threshold=25.0):
        # Observation space matches simulation
        self.observation_space = gym.spaces.Box(
            low=np.array([-max_position, -5.0, -np.radians(max_angle), -np.radians(100)]),
            high=np.array([max_position, 5.0, np.radians(max_angle), np.radians(100)]),
            dtype=np.float32
        )
        
        # Action space matches simulation
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0]),
            high=np.array([1.0]),
            dtype=np.float32
        )
        
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.hardware = HardwareInterface()

    def step(self, action):
        self.current_step += 1
        action = np.clip(action, -1.0, 1.0)[0]
        
        # Execute on hardware
        self.hardware.execute_action(action)
        
        # Get observation
        obs = self.hardware.get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(obs, action)
        
        # Check termination
        done = self._safety_check(obs)
        truncated = self.current_step >= self.max_episode_steps
        
        return obs, reward, done, truncated, {}

    def _calculate_reward(self, obs, action):
        x, x_dot, theta, theta_dot = obs
        return (1.0 - 0.1*abs(x) - 0.5*abs(theta) 
                - 0.01*(x_dot**2) - 0.01*(theta_dot**2) 
                - 0.001*(action**2))

    def _safety_check(self, obs):
        x, _, theta, _ = obs
        return (abs(x) > 0.45 or 
                abs(theta) > np.radians(20) or
                self.hardware.emergency_stop)

    def reset(self, seed=None, options=None):
        self.hardware.safe_reset()
        self.current_step = 0
        return self.hardware.get_observation(), {}
