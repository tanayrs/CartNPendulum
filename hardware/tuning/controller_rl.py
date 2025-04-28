# controller_rl.py
import os
import numpy as np
from stable_baselines3 import PPO
import gymnasium as gym

class HardwareModelAgent:
    def __init__(self, model_type='PPO', model_path=None, env_name=None, filter_alpha=0.5):
        """
        Hardware deployment-ready agent for trained RL models with low-pass filtering

        Args:
            model_type: 'PPO' (case-sensitive)
            model_path: Absolute path to .zip model file
            env_name: Gymnasium environment ID used during training
            filter_alpha: Filter coefficient (0-1), lower values create stronger smoothing
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at {model_path}")

        self.env = gym.make(env_name)
        if model_type == 'PPO':
            self.model = PPO.load(model_path, env=self.env)
        else:
            raise ValueError("Only PPO is supported in this configuration.")

        # Low-pass filter parameters
        self.filter_alpha = filter_alpha
        self.prev_output = 0

    def predict_action(self, state_vector):
        """Hardware-friendly inference method"""
        return int(self.model.predict(state_vector, deterministic=True)[0])

    def control(self, state_vector):
        """Control method with low-pass filtering"""
        action = self.predict_action(state_vector)
        raw_output = ((action - 20) / 20) * 40000

        # Apply low-pass filter: y[n] = α * x[n] + (1-α) * y[n-1]
        filtered_output = self.filter_alpha * raw_output + (1 - self.filter_alpha) * self.prev_output
        self.prev_output = filtered_output

        # Safety: zero output if angular velocity is small
        if -0.2 < state_vector[3] < 0.2:
            filtered_output = 0

        return int(filtered_output)

    def reset_filter(self):
        self.prev_output = 0

    def close(self):
        self.env.close()
