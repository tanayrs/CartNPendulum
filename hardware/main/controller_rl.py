import os
from stable_baselines3 import PPO, DQN
import gymnasium as gym

class HardwareModelAgent:
    def __init__(self, model_type='PPO', model_path=None, env_name=None):
        """
        Hardware deployment-ready agent for trained RL models
        
        Args:
            model_type: 'PPO' or 'DQN' (case-sensitive)
            model_path: Absolute path to .zip model file
            env_name: Gymnasium environment ID used during training
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at {model_path}")
            
        self.env = gym.make(env_name)  # Required for model compatibility
        self.model = PPO.load(model_path, env=self.env) if model_type == 'PPO' \
            else DQN.load(model_path, env=self.env)

    def predict_action(self, state_vector):
        """Hardware-friendly inference method
        Args:
            state_vector: Current environment state as numpy array
        Returns:
            int: Model's predicted action
        """
        return int(self.model.predict(state_vector, deterministic=True)[0])
    
    def control(self, state_vector):
        """Control method to be called in the main loop
        Args:
            state_vector: Current environment state as numpy array
        Returns:
            int: Control Output (PWM)
        """
        action =  self.predict_action(state_vector)
        control_output = ((action-5)/5) * 40000
        return control_output

    def close(self):
        """Close the environment"""
        self.env.close()