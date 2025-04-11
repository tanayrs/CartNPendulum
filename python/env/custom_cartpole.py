# env/custom_cartpole.py

import gymnasium as gym
import numpy as np
# CHANGED: Inherit directly from the built-in CartPoleEnv for baseline functionality.
from gymnasium.envs.classic_control.cartpole import CartPoleEnv

class CustomCartPoleEnv(CartPoleEnv):
    def __init__(self):
        # CHANGED: Call the parent initializer.
        super().__init__()

        # CHANGED: Define custom parameters using MATLAB syntax variable names and values.
        self.p_m = 0.841           # kg, mass of the pole
        self.p_d = 0.2             # m, distance to center-of-mass (CoM) of the pole
        self.p_dw = 0.075          # Custom parameter (e.g., wheel distance)
        self.p_R_wheel = 0.05      # m, radius of the wheel
        self.p_g = 10              # gravity (m/s^2)
        self.p_time_scale = 0.1    # time scale factor for integration
        self.p_rated_torque = 25 * self.p_g / 100  # Rated Torque (calculated from gravity)
        self.Ig = 0                # Mass Moment of Inertia about mass (if applicable)
        self.p_I = (self.p_m * (self.p_d ** 2)) + self.Ig  # Total inertia about the center-of-mass
        self.p_b = 0.4             # Viscous drag coefficient

        # CHANGED: Override key parameters of the original Gymnasium CartPole with custom values.
        self.gravity = self.p_g          # Set gravity to custom value.
        self.masspole = self.p_m         # Use custom pole mass.
        self.masscart = 1.0              # Retaining the default cart mass (can be modified if needed).
        self.total_mass = self.masspole + self.masscart
        self.length = self.p_d           # Use custom length (distance to CoM).
        self.polemass_length = self.masspole * self.length
        self.tau = self.p_time_scale     # Use custom time scale for integration.

    def step(self, action):
        # CHANGED: Redefine the step method to inject custom viscous drag and use the custom parameters.
        x, x_dot, theta, theta_dot = self.state
        force = self.force_mag if action == 1 else -self.force_mag

        costheta = np.cos(theta)
        sintheta = np.sin(theta)

        # CHANGED: Added viscous drag effect (self.p_b) into the dynamics computation.
        temp = (force - self.p_b * x_dot + self.polemass_length * theta_dot**2 * sintheta) / self.total_mass
        thetaacc = (self.gravity * sintheta - costheta * temp) / (
            self.length * (4.0 / 3.0 - self.masspole * costheta**2 / self.total_mass)
        )
        xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass

        # CHANGED: Update the state using the custom time scale (tau).
        x = x + self.tau * x_dot
        x_dot = x_dot + self.tau * xacc
        theta = theta + self.tau * theta_dot
        theta_dot = theta_dot + self.tau * thetaacc

        self.state = (x, x_dot, theta, theta_dot)

        # Use the default termination thresholds defined in the parent class.
        terminated = (
            x < -self.x_threshold
            or x > self.x_threshold
            or theta < -self.theta_threshold_radians
            or theta > self.theta_threshold_radians
        )
        terminated = bool(terminated)

        # CHANGED: Reward is 1.0 per timestep until termination.
        reward = 1.0 if not terminated else 0.0

        return np.array(self.state, dtype=np.float32), reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        # CHANGED: Override the reset method to reinitialize state if needed.
        # Calling the parent reset method for basic setup (seed, etc.)
        observation, info = super().reset(seed=seed, options=options)
        # CHANGED: Reset our state explicitly; here, we set it to zeros for demonstration.
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        return self.state, info
