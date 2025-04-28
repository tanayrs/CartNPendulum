# RL Controler Class, needed to get actions from RL model

import os
import numpy as np
from stable_baselines3 import PPO, DQN
import gymnasium as gym

class HardwareModelAgent:
    def __init__(self, model_type='PPO', model_path=None, env_name=None, filter_alpha=0.4):
        """
        Hardware deployment-ready agent for trained RL models with low-pass filtering

        Args:
            model_type: 'PPO' or 'DQN' (case-sensitive)
            model_path: Absolute path to .zip model file
            env_name: Gymnasium environment ID used during training
            filter_alpha: Filter coefficient (0-1), lower values create stronger smoothing
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at {model_path}")

        self.env = gym.make(env_name)  # Required for model compatibility
        self.model = PPO.load(model_path, env=self.env) if model_type == 'PPO' \
            else DQN.load(model_path, env=self.env)
        
        # Low-pass filter parameters
        self.filter_alpha = filter_alpha  # Filter coefficient (0-1)
        self.prev_output = 0  # Previous filtered output

    def predict_action(self, state_vector):
        """Hardware-friendly inference method
        Args:
            state_vector: Current environment state as numpy array
        Returns:
            int: Model's predicted action
        """
        return int(self.model.predict(state_vector, deterministic=True)[0])

    def control(self, state_vector):
        """Control method with low-pass filtering
        Args:
            state_vector: Current environment state as numpy array
        Returns:
            int: Filtered Control Output (PWM)
        """
        action = self.predict_action(state_vector)
        raw_output = ((action-20)/20) * 40000
        
        # Apply low-pass filter: y[n] = α * x[n] + (1-α) * y[n-1]
        filtered_output = self.filter_alpha * raw_output + (1 - self.filter_alpha) * self.prev_output
        self.prev_output = filtered_output

        if -12 < state_vector[3] < 12:
            filtered_output = 0
        
        return int(filtered_output)
    
    def reset_filter(self):
        """Reset the filter state"""
        self.prev_output = 0
        
    def close(self):
        """Close the environment"""
        self.env.close()
