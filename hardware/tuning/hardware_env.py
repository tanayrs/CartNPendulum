import gymnasium as gym
import numpy as np
from gymnasium import spaces
from imu import MPU6050
from motor import HardwarePWMMotor
from encoder import EncoderProcessor, PiEncoder
import time

class HardwareBalancingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, max_steps=1000):
        super(HardwareBalancingEnv, self).__init__()
        self.imu = MPU6050(roll_offset=0)
        self.motor = HardwarePWMMotor()
        self.encoder = PiEncoder(pin_a=23, pin_b=24)
        self.processor = EncoderProcessor(pulses_per_rev=2262)
        
        self.action_space = spaces.Discrete(41)
        self.observation_space = spaces.Box(
            low=np.array([-2.4, -np.inf, -0.418, -np.inf]),
            high=np.array([2.4, np.inf, 0.418, np.inf]),
            dtype=np.float32
        )
        self.max_steps = max_steps
        self.current_step = 0
        self.safety_threshold = 0.3

    def _get_obs(self):
        ticks = self.encoder.read()
        self.processor.update(ticks)
        imu_data = self.imu.update()
        return np.array([
            self.processor.get_position_meters(),
            self.processor.get_speed_ms(),
            np.deg2rad(imu_data['angle']['roll']),
            np.deg2rad(imu_data['gyro']['x'])
        ], dtype=np.float32)

    def reset(self, seed=None, options=None):
        self.motor.stop()
        self.current_step = 0
        input("Manually position robot upright, then press Enter...")
        return self._get_obs(), {}

    def step(self, action):
        self.current_step += 1
        control_output = ((action-20)/20) * 40000
        self.motor.set_speed(control_output, self.processor.get_speed_ms())
        time.sleep(0.01)
        obs = self._get_obs()
        reward = self._calculate_reward(obs, action)
        terminated = abs(obs[0]) > self.safety_threshold or abs(obs[2]) > 0.2095
        truncated = self.current_step >= self.max_steps
        return obs, reward, terminated, truncated, {}

    def _calculate_reward(self, obs, action):
        theta = obs[2]
        action_penalty = (action - 20)**2 * 0.001
        return (1 - abs(theta)) - action_penalty

    def close(self):
        self.motor.cleanup()
        self.encoder.cleanup()
