"""
This script creates a custom CartPole environment with modified parameters for our problem.
The original CartPole environment is based on the classic control problem where a pole is balanced on a cart.
The custom environment allows for more flexibility in the dynamics of the system.
"""

"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from http://incompleteideas.net/sutton/book/code/pole.c
permalink: https://perma.cc/C9ZM-652R
"""

#imports
import math
import csv
from typing import Optional, Tuple, Union
import numpy as np
import gymnasium as gym
from gymnasium import logger, spaces
from gymnasium.envs.classic_control import utils
from gymnasium.error import DependencyNotInstalled

# we removed the import of VectorEnv and AutoresetMode as they are not used in this code, and other VectorEnv related code too


class CartPoleEnv(gym.Env[np.ndarray, Union[int, np.ndarray]]):
    """
    Description:
    A custom cart-pole system based on the classic control problem.
    A pole is attached by an un-actuated joint to a cart, which moves along a track.
    The pendulum starts upright, and the goal is to balance the pole by applying forces
    to the cart.

    ## Action Space
    The action is a `Discrete(41)` type, allowing for 41 different force values:
    - Values range from 0 to 40
    - 20 represents no force
    - Values < 20 push the cart left (with increasing magnitude)
    - Values > 20 push the cart right (with increasing magnitude)
    - Analogous to Voltage Values from -12V to +12V in 0.6 V Increments

    The force is calculated as: force = (force_mag/20)*(action-20) based on the rated torque of the motor.

    ## Observation Space
    The observation is a `ndarray` with shape `(4,)` with the values corresponding to:

    | Num | Observation           | Min                 | Max               |
    |-----|-----------------------|---------------------|-------------------|
    | 0   | Cart Position         | -4.8                | 4.8               |
    | 1   | Cart Velocity         | -Inf                | Inf               |
    | 2   | Pole Angle            | ~ -0.21 rad (-12°)  | ~ 0.21 rad (12°)  |
    | 3   | Pole Angular Velocity | -Inf                | Inf               |

    ## Rewards
    Two reward systems are implemented:
    1. State-action based reward (default): 
       - Reward = math.cos(theta)^2 - lambda_theta * action_penalty
       - Rewards proper balancing while penalizing excessive actions
    
    2. Sutton-Barto reward (optional):
       - +1 for each step if the pole is upright
       - -1 when the episode terminates due to pole falling

    ## Starting State
    All observations are assigned a uniformly random value in `(-0.2, 0.2)`

    ## Episode End
    The episode ends if any one of the following occurs:
    1. Termination: Pole Angle is greater than ±12°
    2. Termination: Cart Position is greater than ±2.4
    3. Truncation: Episode length exceeds 500 steps

    ## Custom Physics Parameters
    This implementation uses custom physical parameters:
    - Pole mass: 0.841 kg
    - Distance to center-of-mass: 0.2 m
    - Custom gravity: 10 m/s²
    - Custom time scale: 0.01 (1 ms)
    - Custom viscous drag coefficient: 0.4
    
    The dynamics are implemented using detailed equations of motion that account for
    these physical parameters.

    ## Arguments
    | Parameter             | Type     | Default | Description                                      |
    |-----------------------|----------|---------|--------------------------------------------------|
    | `sutton_barto_reward` | **bool** | `False` | Use Sutton-Barto reward instead of state-action  |
    | `render_mode`         | **str**  | `None`  | Rendering mode ('human' or 'rgb_array')          |
    """

    metadata = {
        "render_modes": ["human", "rgb_array"],  # Available rendering modes: human for live display, rgb_array for numpy array
        "render_fps": 24,  # Frames per second for display when rendering
    }

    def __init__(
        self, sutton_barto_reward: bool = False, render_mode: Optional[str] = None,
        reward_type: str = "state_action"
    ):
        self.reward_type = reward_type
        #self._sutton_barto_reward = sutton_barto_reward
        #self.state_action_reward = True  # If True, the reward is based on the state and action taken
        self.k = 0.1  # Hyperparameter for action penalty in reward function

        self.max_steps = 499  # Maximum number of steps in an episode
        self.step_counter = 0  # Counter for steps taken in the current episode
        self.global_step_counter = 0  # Global step counter for the entire training session

        # Logging state and rewards
        self.logging_enabled = True  # You can disable it if needed
        self.log_file = "Training/Logs/reward_state_log.csv"
        if self.logging_enabled:
            # Overwrite existing file and write headers
            with open(self.log_file, mode="w", newline="\n") as f:
                writer = csv.writer(f)
                writer.writerow(["step", "episode_step", "reward", "x", "x_dot", "theta", "theta_dot"])

        # CHANGED: Override key parameters of the original Gymnasium CartPole with custom values.
        self.gravity = 10         # Set gravity to custom value.
        self.masspole = 0.841           # kg, mass of the pole
        self.masscart = 1.0              # Retaining the default cart mass (can be modified if needed).
        self.total_mass = self.masspole + self.masscart
        self.length = 0.2             # m, distance to center-of-mass (CoM) of the pole
        self.p_dw = 0.075          # Custom parameter (e.g., wheel distance)
        self.p_R_wheel = 0.05      # m, radius of the wheel
        self.p_rated_torque = 25 * self.gravity / 100  # Rated Torque (calculated from gravity)
        self.Ig = 0                # Mass Moment of Inertia about mass (if applicable)
        self.p_I = (self.masspole * (self.length** 2)) + self.Ig  # Total inertia about the center-of-mass
        self.p_b = 0.4     
        self.polemass_length = self.masspole * self.length
        self.force_mag = self.p_rated_torque / self.p_R_wheel  # Use custom force magnitude
        self.tau = 0.01  # seconds between state updates
        self.kinematics_integrator = "semi-implicit euler"

        # Angle at which to fail the episode
        self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 2.4

        # Angle limit set to 2 * theta_threshold_radians so failing observation, is still within bounds.
        high = np.array(
            [
                self.x_threshold * 2,
                np.inf,
                self.theta_threshold_radians * 2,
                np.inf,
            ],
            dtype=np.float32,
        )

        # our pwm values go from -40000 to 40000, but that maps to 12V, realistically you can only really 
        # 5.5/0.5 = 11 states, so we can use 0.5V as the low and high values
        self.action_space = spaces.Discrete(41)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.render_mode = render_mode

        self.screen_width = 600
        self.screen_height = 400
        self.screen = None
        self.clock = None
        self.isopen = True
        self.state: np.ndarray | None = None
        self.steps_beyond_terminated = None

    def step(self, action):
        # Validate that the action is within the defined action space
        assert self.action_space.contains(
            action
        ), f"{action!r} ({type(action)}) invalid"
        # Ensure environment has been reset before stepping
        assert self.state is not None, "Call reset before using step method."
        # Extract individual state components
        x, x_dot, theta, theta_dot = self.state
        
        # Calculate force applied to cart based on action (scaled linearly around the midpoint 20)
        force = (self.force_mag/20)*(action-20)

        # print(f'Custom step function called with {action=}')

        # Define physical constants for equations of motion
        d = self.length           # Distance to center of mass of pole
        m = self.masscart         # Mass of the cart
        g = self.gravity          # Gravitational acceleration
        b = self.p_b              # Viscous drag coefficient
        I = self.p_I              # Moment of inertia of the pole

        # Calculate cart acceleration (xacc) using the equation of motion derived from Lagrangian mechanics
        xacc = (force*I - I*x_dot*b + force*(d**2)*m - x_dot*b*(d**2)*m + ((d**2)*g*(m**2)*math.sin(2*theta))/2 - 
       (d**3)*(m**2)*(theta_dot**2)*math.sin(theta) - I*d*m*(theta_dot**2)*math.sin(theta))/(m*(I + (d**2)*m - (d**2)*m*(math.cos(theta)**2)))

        # Calculate angular acceleration of the pole (thetaacc) using the equation of motion
        thetaacc = (d*(- d*m*math.cos(theta)*math.sin(theta)*(theta_dot**2) + force*math.cos(theta) + g*m*math.sin(theta) - b*x_dot*math.cos(theta)))/(I + (d**2)*m*(math.sin(theta)**2))


        if self.kinematics_integrator == "semi-implicit euler":  # semi-implicit euler
            x_dot = x_dot + self.tau * xacc
            x = x + self.tau * x_dot
            theta_dot = theta_dot + self.tau * thetaacc
            theta = theta + self.tau * theta_dot
        else: # euler
            x = x + self.tau * x_dot
            x_dot = x_dot + self.tau * xacc
            theta = theta + self.tau * theta_dot
            theta_dot = theta_dot + self.tau * thetaacc


        self.state = np.array((x, x_dot, theta, theta_dot), dtype=np.float64)

        terminated = bool(
            x < -self.x_threshold
            or x > self.x_threshold
            or theta < -self.theta_threshold_radians
            or theta > self.theta_threshold_radians
        )

        truncated = self.step_counter >= self.max_steps

        if not terminated:
            if self.reward_type == "state_action":
                # Use custom state-action reward
                action_penalty = (action - 20)**2
                lambda_theta = self.k * (1 - abs(math.cos(theta)))
                reward = math.cos(theta)**2 - lambda_theta * action_penalty
            else:
                reward = 0.0 #if self._sutton_barto_reward else 1.0

        elif self.steps_beyond_terminated is None:
            # Pole just fell!
            self.steps_beyond_terminated = 0
            reward = -1.0 #if self._sutton_barto_reward else 1.0

        else:
            if self.steps_beyond_terminated == 0:
                logger.warn(
                    "You are calling 'step()' even though this environment has already returned terminated = True. "
                    "You should always call 'reset()' once you receive 'terminated = True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_terminated += 1
            reward = -1.0 if self._sutton_barto_reward else 0.0

        if self.render_mode == "human":
            self.render()
        
        # Log reward and state if logging is enabled
        if self.logging_enabled:
            with open(self.log_file, mode="a", newline="\n") as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.global_step_counter,
                    self.step_counter,
                    reward,
                    self.state[0],  # x
                    self.state[1],  # x_dot
                    self.state[2],  # theta
                    self.state[3]   # theta_dot
                ])

        self.step_counter += 1
        self.global_step_counter += 1
        return np.array(self.state, dtype=np.float32), reward, terminated, truncated, {}

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        # Initialize random number generator with the provided seed
        super().reset(seed=seed)
        
        # Parse reset bounds from options or use defaults (-0.2, 0.2)
        # These bounds determine the initial randomization of the environment state
        low, high = utils.maybe_parse_reset_bounds(
            options, -0.2, 0.2  # default low - small random initial values
        )  # default high - prevents large initial deviations
        
        # Initialize state with random values: [cart_position, cart_velocity, pole_angle, pole_angular_velocity]
        # Small initial values ensure the pendulum starts near upright position
        self.state = self.np_random.uniform(low=low, high=high, size=(4,))
        
        # Reset termination tracking variable
        self.steps_beyond_terminated = None

        # Display initial state if human rendering is enabled
        if self.render_mode == "human":
            self.render()

        # Reset step counter for new episode
        self.step_counter = 0
        
        # Return initial observation and empty info dictionary
        return np.array(self.state, dtype=np.float32), {}

    def render(self):
        # Return if no render mode is specified
        if self.render_mode is None:
            assert self.spec is not None
            gym.logger.warn(
                "You are calling render method without specifying any render mode. "
                "You can specify the render_mode at initialization, "
                f'e.g. gym.make("{self.spec.id}", render_mode="rgb_array")'
            )
            return

        # Import pygame for rendering, raise error if not installed
        try:
            import pygame
            from pygame import gfxdraw
        except ImportError as e:
            raise DependencyNotInstalled(
                'pygame is not installed, run `pip install "gymnasium[classic-control]"`'
            ) from e

        # Initialize pygame and create screen if not already done
        if self.screen is None:
            pygame.init()
            if self.render_mode == "human":
                # Create visible window for human viewing
                pygame.display.init()
                self.screen = pygame.display.set_mode(
                    (self.screen_width, self.screen_height)
                )
            else:  # mode == "rgb_array"
                # Create offscreen surface for array output
                self.screen = pygame.Surface((self.screen_width, self.screen_height))
        
        # Initialize clock for controlling frame rate if not already done
        if self.clock is None:
            self.clock = pygame.time.Clock()

        # Calculate scaling parameters
        world_width = self.x_threshold * 2  # Total width of the world in physics units
        scale = self.screen_width / world_width  # Conversion factor from physics to pixels
        polewidth = 10.0  # Width of the pole in pixels
        polelen = scale * (2 * self.length)  # Length of the pole in pixels (scaled from physics)
        cartwidth = 50.0  # Width of the cart in pixels
        cartheight = 20.0  # Height of the cart in pixels

        # Return None if state is not initialized
        if self.state is None:
            return None

        # Extract state vector
        x = self.state
        
        # Initialize rendering surface if needed
        self.surf = pygame.Surface((self.screen_width, self.screen_height))
        
        # Fill background with white
        self.surf.fill((255, 255, 255))

        # Define cart coordinates
        l, r, t, b = -cartwidth / 2, cartwidth / 2, cartheight / 2, -cartheight / 2  # Left, right, top, bottom
        axleoffset = cartheight / 4.0  # Offset for the axle from top of cart
        cartx = x[0] * scale + self.screen_width / 2.0  # Calculate cart x position (physics to pixels)
        carty = 100  # Fixed cart y position (top of cart)
        
        # Draw the cart as a black rectangle
        cart_coords = [(l, b), (l, t), (r, t), (r, b)]  # Corners of the cart
        cart_coords = [(c[0] + cartx, c[1] + carty) for c in cart_coords]  # Translate to correct position
        gfxdraw.aapolygon(self.surf, cart_coords, (0, 0, 0))  # Draw anti-aliased outline
        gfxdraw.filled_polygon(self.surf, cart_coords, (0, 0, 0))  # Fill with black

        # Define pole coordinates (brown rectangle)
        l, r, t, b = (
            -polewidth / 2,
            polewidth / 2,
            polelen - polewidth / 2,
            -polewidth / 2,
        )

        # Calculate pole position accounting for rotation
        pole_coords = []
        for coord in [(l, b), (l, t), (r, t), (r, b)]:
            coord = pygame.math.Vector2(coord).rotate_rad(-x[2])  # Rotate by negative of pole angle
            coord = (coord[0] + cartx, coord[1] + carty + axleoffset)  # Position relative to cart axle
            pole_coords.append(coord)
        
        # Draw the pole (brown color)
        gfxdraw.aapolygon(self.surf, pole_coords, (202, 152, 101))  # Anti-aliased outline
        gfxdraw.filled_polygon(self.surf, pole_coords, (202, 152, 101))  # Fill with brown

        # Draw the axle as a blue circle
        gfxdraw.aacircle(
            self.surf,
            int(cartx),
            int(carty + axleoffset),
            int(polewidth / 2),
            (129, 132, 203),
        )
        gfxdraw.filled_circle(
            self.surf,
            int(cartx),
            int(carty + axleoffset),
            int(polewidth / 2),
            (129, 132, 203),
        )

        # Draw the ground as a black horizontal line
        gfxdraw.hline(self.surf, 0, self.screen_width, carty, (0, 0, 0))

        # Flip the surface vertically to match the physics coordinates (y-up)
        self.surf = pygame.transform.flip(self.surf, False, True)
        
        # Copy the rendered scene to the display surface
        self.screen.blit(self.surf, (0, 0))
        
        # Handle specific render mode outputs
        if self.render_mode == "human":
            # Process events, maintain framerate, and update the window
            pygame.event.pump()
            self.clock.tick(self.metadata["render_fps"])
            pygame.display.flip()

        elif self.render_mode == "rgb_array":
            # Return the rendered frame as a numpy array for RL algorithms
            return np.transpose(
                np.array(pygame.surfarray.pixels3d(self.screen)), axes=(1, 0, 2)
            )

    def close(self):
        # Clean up resources when environment is closed
        if self.screen is not None:
            import pygame  # Import pygame for cleanup
            
            pygame.display.quit()  # Close the display window
            pygame.quit()  # Terminate all pygame modules
            self.isopen = False  # Mark environment as closed