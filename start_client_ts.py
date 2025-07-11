#Copyright(C) 2023 Intel Corporation
#SPDX-License-Identifier: Apache-2.0
#File : start_client_demo.py

from network_gym_client import load_config_file
from network_gym_client import Env as NetworkGymEnv
import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({
    "font.size": 14,
    "axes.labelsize": 14,
    "axes.titlesize": 14,
    "legend.fontsize": 14,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "lines.linewidth": 2,
})

# MCS 0 - 11
mcs_Mbps_20mhz = [8.6, 17.2, 25.8, 34.4, 51.6, 68.8, 
                  77.4, 86.0, 103.2, 114.7, 129.0, 143.4]

# statistics per MCS
mcs_alpha = np.zeros(12)
mcs_beta = np.zeros(12)

client_id = 0
env_name = "ts"
config_json = load_config_file(env_name)
config_json["rl_config"]["agent"] = "muyuan_agent"
# Create the environment
env = NetworkGymEnv(client_id, config_json) # make a network env using pass client id and configure file arguments.

num_steps = 100
obs, info = env.reset()

action = np.array([0])  # try MCS 0 first
prev_action = np.array([-1])

mcs_vs_time = np.zeros(num_steps)
last_step = 0

for step in range(num_steps):

    obs, _, terminated, truncated, _ = env.step(action)
    prev_action = action

    if np.sum(obs) == 0:
        continue

    # get new action
    mcs_old = prev_action[0]
    n_succ = obs[0][0]
    n_fail = obs[1][0]
    mcs_alpha[mcs_old] += n_succ
    mcs_beta[mcs_old] += n_fail
    mcs_new = np.argmax(np.random.beta(mcs_alpha + 1, mcs_beta + 1) * mcs_Mbps_20mhz)
    action = np.array([mcs_new])

    mcs_vs_time[step] = mcs_new
    last_step = step

    # If the environment is end, exit
    if terminated:
        break

    # If the episode is up (environment still running), then start another one
    if truncated:
        obs, info = env.reset()

# plot MCS vs timestep
fig, ax = plt.subplots(figsize=(7, 5))
timesteps = np.arange(last_step + 1)
ax.plot(timesteps, mcs_vs_time[0:last_step + 1], 'r-o')
ax.set_ylim(0, 11)
ax.set_xlabel('Time step')
ax.set_ylabel('New MCS index')
plt.savefig("mcs_vs_timestep.pdf", bbox_inches='tight')
