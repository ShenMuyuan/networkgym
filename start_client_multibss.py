#Copyright(C) 2023 Intel Corporation
#SPDX-License-Identifier: Apache-2.0
#File : start_client_demo.py

from network_gym_client import load_config_file
from network_gym_client import Env as NetworkGymEnv
import numpy as np
import random
from collections import namedtuple, deque
import math
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import time

device = torch.device(
    "cuda" if torch.cuda.is_available() else
    ("mps" if torch.backends.mps.is_available() else "cpu")
)
print(f"PyTorch will use device {device}")
time.sleep(1)

client_id = 0
env_name = "multibss"
config_json = load_config_file(env_name)
config_json["rl_config"]["agent"] = "muyuan_agent"
# Create the environment
env = NetworkGymEnv(client_id, config_json) # make a network env using pass client id and configure file arguments.

Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))

class ReplayMemory(object):
    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)

    def push(self, *args):
        """Save a transition"""
        self.memory.append(Transition(*args))

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)

class DQN(nn.Module):
    def __init__(self, n_observations, n_actions):
        super(DQN, self).__init__()
        self.layer1 = nn.Linear(n_observations, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, n_actions)

    def forward(self, x):
        x = F.relu(self.layer1(x))
        x = F.relu(self.layer2(x))
        return self.layer3(x)
    
BATCH_SIZE = 32
GAMMA = 0.99
EPS_START = 0.9
EPS_END = 0.05
EPS_DECAY = 50
TAU = 0.005
LR = 1e-4

# TODO: get these sizes from config to avoid redundant code
n_ap = 4
network_size = 4
n_total = n_ap * (network_size + 1)
state = np.zeros((network_size + 1, n_total + 1))
rewards = []
overall_rewards = []
n_observation = state.size
n_action = env.adapter.get_action_space().nvec[0]
policy_net = DQN(n_observation, n_action).to(device)
target_net = DQN(n_observation, n_action).to(device)
target_net.load_state_dict(policy_net.state_dict())
optimizer = optim.AdamW(policy_net.parameters(), lr=LR, amsgrad=True)
memory = ReplayMemory(200)

steps_done = 0

def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * \
                    math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            return policy_net(state).max(1)[1].view(1, 1)
    else:
        return torch.tensor([[np.random.randint(0, n_action)]], device=device, dtype=torch.long)

def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)
    batch = Transition(*zip(*transitions))
    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                            batch.next_state)), device=device, dtype=torch.bool)
    non_final_next_states = torch.cat([s for s in batch.next_state
                                       if s is not None])
    state_batch = torch.cat(batch.state)
    action_batch = torch.cat(batch.action)
    reward_batch = torch.cat(batch.reward)
    state_action_values = policy_net(state_batch).gather(1, action_batch)

    next_state_values = torch.zeros(BATCH_SIZE, device=device)
    with torch.no_grad():
        next_state_values[non_final_mask] = target_net(non_final_next_states).max(1)[0]
    expected_state_action_values = (next_state_values * GAMMA) + reward_batch

    # print("state_action_values:", state_action_values)
    # print("expected_state_action_values:", expected_state_action_values)

    criterion = nn.SmoothL1Loss()
    # loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))
    loss = criterion(state_action_values, expected_state_action_values[:, :1])

    optimizer.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_value_(policy_net.parameters(), 100)
    optimizer.step()

num_steps = 1000
obs, info = env.reset()

for step in range(num_steps):

    state[:, :n_total] = obs[0][:network_size+1, :n_total]
    state[:, -1] = obs[1][:network_size+1]
    cur_state = torch.tensor(state.reshape(1, -1)[0], dtype=torch.float32, device=device).unsqueeze(0)

    if step == 0:
        prev_state = cur_state
        action = torch.tensor([[0]], device=device, dtype=torch.long)
    else:
        rewards.append(reward)
        reward = torch.tensor([reward], device=device)
        memory.push(prev_state, action, cur_state, reward)
        prev_state = cur_state
        action = select_action(cur_state)
        optimize_model()
        target_net_state_dict = target_net.state_dict()
        policy_net_state_dict = policy_net.state_dict()
        for key in policy_net_state_dict:
            target_net_state_dict[key] = policy_net_state_dict[key]*TAU + target_net_state_dict[key]*(1-TAU)
        target_net.load_state_dict(target_net_state_dict)

    action_int = action.cpu().numpy()[0][0] + (-82)
    obs, reward, terminated, truncated, info = env.step(np.array([action_int]))

    # If the environment is end, exit
    if terminated:
        break

    # If the episode is up (environment still running), then start another one
    if truncated:
        obs, info = env.reset()

