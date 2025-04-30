import gymnasium as gym
import numpy as np
from gymnasium import spaces
from imu import MPU6050
from motor import HardwarePWMMotor
from encoder import EncoderProcessor, PiEncoder
import time

class HardwareBalancingEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, max_steps=1000, k = 0.1):
        super(HardwareBalancingEnv, self).__init__()
        # Hardware initialization
        self.imu = MPU6050(roll_offset=5)
        self.motor = HardwarePWMMotor()
        self.encoder = PiEncoder(pin_a=23, pin_b=24)
        self.processor = EncoderProcessor(pulses_per_rev=2262)
        # Action and observation spaces
        self.action_space = spaces.Discrete(41) # Same as simulation
        self.observation_space = spaces.Box(
            low=np.array([-4.8, -np.inf, -0.41887903, -np.inf]),
            high=np.array([4.8, np.inf, 0.41887903, np.inf]),
            dtype=np.float32
        )
        self.max_steps = max_steps
        self.current_step = 0
        self.safety_threshold = 0.3 # meters
        self.k = k

    def _get_obs(self):
        ticks = self.encoder.read()
        self.processor.update(ticks)
        imu_data = self.imu.update()
        # CRITICAL FIX: imu_data['angle']['roll'] is in degrees, convert to radians ONCE
        theta = imu_data['angle']['roll']
        theta_dot = imu_data['gyro']['x']

        return np.array([
            self.processor.get_position_meters(),  # x
            self.processor.get_speed_ms(),         # x_dot
            theta,                                # theta (radians)
            theta_dot                             # theta_dot (radians/sec)
        ], dtype=np.float32)
    
    def reset(self, seed=None, options=None):
        self.motor.stop()
        self.current_step = 0
        self.imu.close()
        self.imu = MPU6050(roll_offset=5)
        self.encoder = PiEncoder(pin_a=23, pin_b=24)
        self.processor = EncoderProcessor(pulses_per_rev=2262)
    
        # Wait for manual positioning and ensure stability
        input("Manually position robot upright, then press Enter...")
    
        # Check if the robot is stable before proceeding
        stable = False
    
        while not stable:
            obs = self._get_obs()
            angle = obs[2]
        
            if abs(angle) < 0.1:  # Much stricter threshold for starting
                stable = True
                print("Robot is stable, starting training...")
            else:
                print(f"Robot not stable: angle = {angle} radians ({angle * 57.3} degrees)")
                print(f"Please adjust position.")
                input("Press Enter when ready...")
    
        return self._get_obs(), {}

    def step(self, action):
        self.current_step += 1
        # Convert action to motor command
        control_output = ((action-20)/20) * 40000
        print(f"Action: {action}, Control Output: {control_output}, Current Speed: {self.processor.get_speed_ms()}")
        self.motor.set_speed(control_output, self.processor.get_speed_ms())
        time.sleep(0.01)
        obs = self._get_obs()
        reward = self._calculate_reward(obs, action)
        # Terminate if out of bounds (position or angle in radians)
        terminated = abs(obs[0]) > self.safety_threshold or abs(obs[2]) > 0.2095 # ±12°
        truncated = self.current_step >= self.max_steps
        return obs, reward, terminated, truncated, {}

    def _calculate_reward(self, obs, action):
        theta = obs[2]
        cos2theta = np.cos(theta)**2
        action_penalty = (action - 20)**2 * self.k * (1-abs(np.cos(theta)))
        return cos2theta - action_penalty

    def close(self):
        self.motor.cleanup()
        self.encoder.cleanup()
