# File: controller_rl.py (updated)
import os
import numpy as np
from stable_baselines3 import PPO, DQN, SAC
from stable_baselines3.common.buffers import ReplayBuffer
import gymnasium as gym

class RealWorldTrainingAgent:
    def __init__(self, 
                 model_type='PPO',
                 policy='MlpPolicy',
                 env_config=None,
                 buffer_size=100000,
                 learning_starts=1000,
                 batch_size=256,
                 tau=0.005,
                 gamma=0.99,
                 target_update_interval=1):
        
        self.env = RealWorldEnv(**env_config)
        self.model = self._init_model(model_type, policy)
        self.buffer = ReplayBuffer(
            buffer_size,
            self.env.observation_space,
            self.env.action_space,
            device="cpu"
        )
        self.batch_size = batch_size
        self.tau = tau
        self.gamma = gamma
        self.target_update_interval = target_update_interval
        self.learning_starts = learning_starts
        self.total_timesteps = 0
        self.episode_reward = 0
        self.episode_length = 0

    def _init_model(self, model_type, policy):
        if model_type == 'PPO':
            return PPO(policy, self.env, verbose=1)
        elif model_type == 'SAC':
            return SAC(policy, self.env, verbose=1,
                      buffer_size=self.buffer_size,
                      learning_starts=self.learning_starts,
                      batch_size=self.batch_size,
                      tau=self.tau,
                      gamma=self.gamma,
                      target_update_interval=self.target_update_interval)
        else:
            raise ValueError(f"Unsupported model type: {model_type}")

    def collect_experience(self, total_timesteps):
        obs, _ = self.env.reset()
        for _ in range(total_timesteps):
            action, _ = self.model.predict(obs, deterministic=False)
            next_obs, reward, done, truncated, info = self.env.step(action)
            self.buffer.add(obs, next_obs, action, reward, done, [info])
            
            if self.total_timesteps > self.learning_starts:
                self.model.replay_buffer.add(obs, next_obs, action, reward, done, [info])
                
            obs = next_obs
            self.total_timesteps += 1
            self.episode_reward += reward
            self.episode_length += 1
            
            if done or truncated:
                print(f"Episode Reward: {self.episode_reward:.2f}, Length: {self.episode_length}")
                obs, _ = self.env.reset()
                self.episode_reward = 0
                self.episode_length = 0

    def train(self, total_timesteps):
        self.model.learn(total_timesteps=total_timesteps,
                         log_interval=4,
                         reset_num_timesteps=False)

    def save_model(self, path):
        self.model.save(path)

class RealWorldEnv(gym.Env):
    def __init__(self, 
                 max_angle=15.0,
                 max_position=0.5,
                 max_episode_steps=500,
                 safety_threshold=25.0):
        
        self.observation_space = gym.spaces.Box(
            low=np.array([-max_position, -5.0, -np.radians(max_angle), -np.radians(100)]),
            high=np.array([max_position, 5.0, np.radians(max_angle), np.radians(100)]),
            dtype=np.float32
        )
        
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0]),
            high=np.array([1.0]),
            dtype=np.float32
        )
        
        self.max_episode_steps = max_episode_steps
        self.safety_threshold = safety_threshold
        self.current_step = 0
        self.hardware_interface = HardwareInterface()

    def step(self, action):
        self.current_step += 1
        
        # Execute action on hardware
        self.hardware_interface.execute_action(action[0])
        
        # Get new observation
        obs = self.hardware_interface.get_observation()
        
        # Calculate reward
        reward = self.calculate_reward(obs, action)
        
        # Check termination conditions
        done = self.check_termination(obs)
        truncated = self.current_step >= self.max_episode_steps
        
        return obs, reward, done, truncated, {}

    def calculate_reward(self, obs, action):
        x, x_dot, theta, theta_dot = obs
        reward = (1.0 - 0.1*abs(x) - 0.5*abs(theta) 
                 - 0.01*(x_dot**2) - 0.01*(theta_dot**2) 
                 - 0.001*(action[0]**2))
        return reward

    def check_termination(self, obs):
        x, _, theta, _ = obs
        return (abs(x) > 0.45 or 
                abs(theta) > np.radians(20) or
                self.hardware_interface.emergency_stop)

    def reset(self, seed=None, options=None):
        self.hardware_interface.safe_reset()
        self.current_step = 0
        return self.hardware_interface.get_observation(), {}

class HardwareInterface:
    def __init__(self):
        from motor import HardwarePWMMotor
        from imu import MPU6050
        from encoder import PiEncoder, EncoderProcessor
        
        self.motor = HardwarePWMMotor()
        self.imu = MPU6050()
        self.encoder = PiEncoder(pin_a=23, pin_b=24)
        self.processor = EncoderProcessor(pulses_per_rev=2262)
        self.emergency_stop = False
        
    def get_observation(self):
        # Read sensor data
        imu_data = self.imu.update()
        encoder_ticks = self.encoder.read()
        self.processor.update(encoder_ticks)
        
        return np.array([
            self.processor.get_position_meters(),
            self.processor.get_speed_ms(),
            np.radians(imu_data['angle']['roll']),
            imu_data['gyro']['x']
        ], dtype=np.float32)

    def execute_action(self, normalized_action):
        # Convert normalized action to PWM signal
        pwm = np.clip(normalized_action * 40000, -40000, 40000)
        self.motor.set_speed(pwm, self.processor.get_speed_ms())
        
        # Safety checks
        if abs(self.processor.get_position_meters()) > 0.5:
            self.emergency_stop = True
        if abs(imu_data['angle']['roll']) > 25:
            self.emergency_stop = True

    def safe_reset(self):
        self.motor.stop()
        self.emergency_stop = False
        # Wait for manual reset by human operator
        input("Reset robot to starting position and press Enter...")
