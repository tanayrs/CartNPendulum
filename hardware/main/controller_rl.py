'''
Reinforcement Learning Controller

This module provides a hardware-friendly interface for deploying trained reinforcement learning models to control the segway system. It handles model loading, state processing, action prediction, and implements signal smoothing for hardware safety.

Features:
- Loads and executes pre-trained PPO or DQN reinforcement learning models
- Implements low-pass filtering to smooth control signals for hardware protection
- Converts discrete actions to continuous control signals suitable for motor control
- Provides special handling for near-zero velocity states to prevent oscillations
- Offers clean reset and shutdown capabilities for the control system

Implementation:
- Loads trained model files (.zip) from stable-baselines3
- Processes state vectors from hardware sensors (position, velocity, angle, angular velocity)
- Applies the learned policy to determine optimal control actions
- Filters the raw control signal for smooth transitions between control outputs
- Scales the output for direct application to motor PWM control

Usage:
    # Initialize controller with trained model
    controller = HardwareModelAgent(
        model_type='PPO',
        model_path='/path/to/trained_model.zip',
        env_name='CustomCartPole-v1',
        filter_alpha=0.3
    )
    
    # In control loop
    while running:
        # Get state from sensors
        state = [x, x_dot, theta, theta_dot]
        
        # Get filtered control output
        control_output = controller.control(state)
        
        # Apply to motor
        motor.set_speed(control_output)
'''

# imports
import os
import numpy as np
from stable_baselines3 import PPO, DQN
import gymnasium as gym

class HardwareModelAgent:
    def __init__(self, model_type='PPO', model_path=None, env_name=None, filter_alpha=0.3):
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
        print(f'{action}')
        raw_output = ((action-20)/20) * 40000
        
        # Apply low-pass filter: y[n] = α * x[n] + (1-α) * y[n-1]
        filtered_output = self.filter_alpha * raw_output + (1 - self.filter_alpha) * self.prev_output
        self.prev_output = filtered_output

        if -0.2 < state_vector[3] < 0.2:
            filtered_output = 0
        
        return int(filtered_output)
    
    def reset_filter(self):
        """Reset the filter state"""
        self.prev_output = 0
        
    def close(self):
        """Close the environment"""
        self.env.close()