import math
import random
import os
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import matplotlib.pyplot as plt

# ================================================
# Continuous CartPole Environment for DDPG
# ================================================
# This environment is adapted from Rich Sutton’s implementation.
# Unlike the standard discrete version, this one accepts a continuous
# action (a float between -1.0 and 1.0).
class CartPole:
    def __init__(self):
        # Physical constants (you can modify these if you like)
        self._cart_mass = 0.31    # kg
        self._pole_mass = 0.055   # kg
        self._pole_length = 0.4   # m

        # Thresholds for termination
        self.x_threshold = 1.0
        self.theta_threshold = 12 * 2 * math.pi / 360  # in radians

        self._state = None
        self._done = True

    def reset(self):
        self._step = 0
        # Initialize state values with small random noise
        self._cart_position = math.tanh(random.gauss(0.0, 0.01)) * 4.8
        self._cart_velocity = random.uniform(-0.05, 0.05)
        initial_pole_angle = random.uniform(-0.05, 0.05)
        self._pole_angle = (initial_pole_angle + math.pi) % (2 * math.pi) - math.pi
        self._pole_angular_velocity = random.uniform(-0.05, 0.05)
        self._state = [self._cart_position, self._cart_velocity, self._pole_angle, self._pole_angular_velocity]
        self._done = False
        return self._state

    def step(self, action: float):
        if self._done:
            raise Exception("Call reset() before step().")
        self._step += 1

        # Add some noise to the action for exploration
        force = 1.0 * (action + random.uniform(-0.02, 0.02))
        total_mass = self._cart_mass + self._pole_mass
        pole_half_length = self._pole_length / 2
        pole_mass_length = self._pole_mass * pole_half_length

        cosTheta = math.cos(self._pole_angle)
        sinTheta = math.sin(self._pole_angle)
        temp = (force + pole_mass_length * self._pole_angular_velocity ** 2 * sinTheta) / total_mass

        angularAccel = (9.8 * sinTheta - cosTheta * temp) / (
            pole_half_length * (4.0 / 3.0 - (self._pole_mass * cosTheta ** 2) / total_mass)
        )
        linearAccel = temp - (pole_mass_length * angularAccel * cosTheta) / total_mass

        # Update the state using Euler integration
        self._cart_position += 0.02 * self._cart_velocity
        self._cart_velocity += 0.02 * linearAccel
        self._pole_angle += 0.02 * self._pole_angular_velocity
        # Normalize pole angle to [-pi, pi]
        self._pole_angle = (self._pole_angle + math.pi) % (2 * math.pi) - math.pi
        self._pole_angular_velocity += 0.02 * angularAccel

        self._state = [self._cart_position, self._cart_velocity, self._pole_angle, self._pole_angular_velocity]

        # Determine if episode terminates (only mark termination if thresholds exceeded)
        term = (self._state[0] < -self.x_threshold or 
                self._state[0] > self.x_threshold or 
                self._state[2] < -self.theta_threshold or 
                self._state[2] > self.theta_threshold)
        term = bool(term)
        trunc = (self._step == 500)
        trunc = bool(trunc)
        self._done = bool(term or trunc)
        # Reward is fixed at 1.0 per step
        return self._state, 1.0, term, trunc, {}

# ================================================
# DDPG Networks and Helpers
# ================================================
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Q-network: approximates Q(s, a)
class QNet(nn.Module):
    def __init__(self, hidden_dim=64):
        super().__init__()
        # Input: state (4 dims) and action (1 dim) --> total 5 dims
        self.hidden = nn.Linear(5, hidden_dim)
        self.output = nn.Linear(hidden_dim, 1)

    def forward(self, s, a):
        # Concatenate state and action
        x = torch.cat((s, a), dim=-1)
        x = self.hidden(x)
        x = F.relu(x)
        x = self.output(x)
        return x

# Instantiate Q-networks
q_origin_model = QNet().to(device)  # Q_phi
q_target_model = QNet().to(device)  # Q_phi'
q_target_model.requires_grad_(False)

# Policy network: approximates the deterministic policy mu(s)
class PolicyNet(nn.Module):
    def __init__(self, hidden_dim=64):
        super().__init__()
        # Input: state (4 dims)
        self.hidden = nn.Linear(4, hidden_dim)
        self.output = nn.Linear(hidden_dim, 1)

    def forward(self, s):
        x = self.hidden(s)
        x = F.relu(x)
        x = self.output(x)
        x = torch.tanh(x)  # Ensure output is in the range [-1, 1]
        return x

mu_origin_model = PolicyNet().to(device)  # mu_theta
mu_target_model = PolicyNet().to(device)  # mu_theta'
mu_target_model.requires_grad_(False)

# Optimizers for critic (Q-network) and actor (Policy network)
gamma = 0.99
opt_q = torch.optim.AdamW(q_origin_model.parameters(), lr=0.0005)
opt_mu = torch.optim.AdamW(mu_origin_model.parameters(), lr=0.0005)

def optimize(states, actions, rewards, next_states, dones):
    # Convert data to torch tensors
    states = torch.tensor(states, dtype=torch.float).to(device)
    actions = torch.tensor(actions, dtype=torch.float).unsqueeze(dim=1).to(device)
    rewards = torch.tensor(rewards, dtype=torch.float).unsqueeze(dim=1).to(device)
    next_states = torch.tensor(next_states, dtype=torch.float).to(device)
    dones = torch.tensor(dones, dtype=torch.float).unsqueeze(dim=1).to(device)

    # Update critic (Q-network)
    opt_q.zero_grad()
    q_current = q_origin_model(states, actions)
    with torch.no_grad():
        next_actions = mu_target_model(next_states)
        q_next = q_target_model(next_states, next_actions)
        q_target = rewards + gamma * (1.0 - dones) * q_next
    loss_q = F.mse_loss(q_current, q_target)
    loss_q.backward()
    opt_q.step()

    # Update actor (policy network)
    opt_mu.zero_grad()
    current_actions = mu_origin_model(states)
    # Freeze Q-network parameters during actor update
    for p in q_origin_model.parameters():
        p.requires_grad = False
    actor_loss = -q_origin_model(states, current_actions).mean()
    actor_loss.backward()
    opt_mu.step()
    for p in q_origin_model.parameters():
        p.requires_grad = True

# Update target networks with soft updates
tau = 0.002
def update_target():
    for param, target_param in zip(q_origin_model.parameters(), q_target_model.parameters()):
        target_param.data.copy_(tau * param.data + (1.0 - tau) * target_param.data)
    for param, target_param in zip(mu_origin_model.parameters(), mu_target_model.parameters()):
        target_param.data.copy_(tau * param.data + (1.0 - tau) * target_param.data)

# ================================================
# Replay Buffer
# ================================================
class ReplayBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []

    def add(self, transition):
        if len(self.buffer) >= self.capacity:
            self.buffer.pop(0)
        self.buffer.append(transition)

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        states = [s for (s, a, r, s_next, d) in batch]
        actions = [a for (s, a, r, s_next, d) in batch]
        rewards = [r for (s, a, r, s_next, d) in batch]
        next_states = [s_next for (s, a, r, s_next, d) in batch]
        dones = [d for (s, a, r, s_next, d) in batch]
        return states, actions, rewards, next_states, dones

    def __len__(self):
        return len(self.buffer)

buffer = ReplayBuffer(capacity=20000)

# ================================================
# Ornstein-Uhlenbeck Noise for Exploration
# ================================================
class OrnsteinUhlenbeckNoise:
    def __init__(self, mu, sigma, theta=0.15, dt=1e-2, x0=None):
        self.theta = theta
        self.mu = mu
        self.sigma = sigma
        self.dt = dt
        self.x0 = x0
        self.reset()

    def __call__(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + \
            self.sigma * np.sqrt(self.dt) * np.random.normal(size=self.mu.shape)
        self.x_prev = x
        return x

    def reset(self):
        self.x_prev = self.x0 if self.x0 is not None else np.zeros_like(self.mu)

ou_noise = OrnsteinUhlenbeckNoise(mu=np.zeros(1), sigma=0.05 * np.ones(1))

def select_action(state):
    state_tensor = torch.tensor(np.expand_dims(state, axis=0), dtype=torch.float).to(device)
    with torch.no_grad():
        action = mu_origin_model(state_tensor).cpu().numpy()[0]
    noise = ou_noise()
    action = action + noise
    return float(np.clip(action, -1.0, 1.0).item())

# ================================================
# Main Training Loop
# ================================================
batch_size = 250
max_episodes = 10000
reward_records = []

env = CartPole()

for episode in range(max_episodes):
    state = env.reset()
    done = False
    episode_reward = 0
    while not done:
        # Select action with exploration noise
        action = select_action(state)
        next_state, reward, term, trunc, _ = env.step(action)
        # Use termination flag only (ignore truncation for learning)
        done_flag = 1.0 if term else 0.0
        buffer.add((state, action, reward, next_state, done_flag))
        episode_reward += reward

        if len(buffer) >= batch_size:
            batch = buffer.sample(batch_size)
            optimize(*batch)
            update_target()

        state = next_state

        done = term or trunc

    reward_records.append(episode_reward)
    print(f"Episode {episode}: Reward = {episode_reward}", end="\r")
    # Stop early if average reward over last 50 episodes exceeds threshold
    if len(reward_records) >= 50 and np.mean(reward_records[-50:]) > 475.0:
        break

print("\nTraining complete.")

# Plot reward history with moving average
plt.figure()
avg_rewards = []
for i in range(len(reward_records)):
    start = max(0, i - 49)
    avg_rewards.append(np.mean(reward_records[start:i+1]))
plt.plot(reward_records, label="Episode Reward")
plt.plot(avg_rewards, label="Average Reward (window=50)")
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.legend()
plt.show()
