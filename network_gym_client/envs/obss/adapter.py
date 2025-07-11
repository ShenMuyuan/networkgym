#Copyright(C) 2023 Intel Corporation
#SPDX-License-Identifier: Apache-2.0
#File : adapter.py

import network_gym_client.adapter

import sys
from gymnasium import spaces
import numpy as np
import math
import time
import pandas as pd
import json
from pathlib import Path
import plotext
from rich.panel import Panel
from rich.layout import Layout
from rich.table import Table
from rich.columns import Columns
from collections import deque

# We need to put the RX power matrix into a 1-D observation
MAX_NUM_NODES_BSS0 = 2 ** 3
MAX_NUM_NODES = 2 ** 5
MAX_NUM_OBSERVATION_IDS = MAX_NUM_NODES * MAX_NUM_NODES_BSS0

REWARD_ALPHA = 1
REWARD_BETA = 5
REWARD_ETA = 1
VR_DELAY_CONSTRAINT_MS = 5
VR_THPT_CONSTRAINT_MBPS = 14.7

class Adapter(network_gym_client.adapter.Adapter):
    """nqos_split env adapter.

    Args:
        Adapter (network_gym_client.adapter.Adapter): base class.
    """
    def __init__(self, config_json):
        """Initialize the adapter.

        Args:
            config_json (json): the configuration file
        """

        super().__init__(config_json)

        self.env = Path(__file__).resolve().parent.name
        self.reward = 0
        if config_json['env_config']['env'] != self.env:
            sys.exit("[ERROR] wrong environment Adapter. Configured environment: " + str(config_json['env_config']['env']) + " != Launched environment: " + str(self.env))

        self.mcs = None
        self.action1_history = deque(maxlen=100)
        self.action2_history = deque(maxlen=100)
        self.total_thpt_history = deque(maxlen=100)
        self.vr_thpt_history = deque(maxlen=100)
        self.vr_delay_history = deque(maxlen=100)
        self.reward_history = deque(maxlen=100)
        self.step_num_history = deque(maxlen=100)

    def get_action_space(self):
        """Get action space for the nqos_split env.

        Returns:
            spaces: action spaces
        """

        # OBSS_PD: min: -82 dBm, max: -62 dBm --> 21 integers
        # TX power: min: 16 dBm, max: 20 dBm --> 5 integers
        return spaces.MultiDiscrete(np.array([21, 5]), start=np.array([-82, 16]), seed=42, dtype=np.int32)

    #consistent with the get_observation function.
    def get_observation_space(self):
        """Get the observation space for nqos_split env.

        Returns:
            spaces: observation spaces
        """

        return spaces.Tuple((
            spaces.Box(low=-100, high=100, shape=(MAX_NUM_NODES_BSS0, MAX_NUM_NODES), dtype=np.float32), # RX power
            spaces.Box(low=0, high=11, shape=(MAX_NUM_NODES_BSS0, ), dtype=np.uint32),     # MCS
            spaces.Box(low=0, high=1000, shape=(MAX_NUM_NODES, ), dtype=np.float32),   # UL throughput
            spaces.Box(low=0, high=10000, shape=(1, ), dtype=np.float32),   # VR node access delay
            spaces.Box(low=-100, high=100, shape=(MAX_NUM_NODES, 2), dtype=np.float32)  # Node location
        ))
    
    def get_observation(self, df):
        """Prepare observation for nqos_split env.

        This function should return the same number of features defined in the :meth:`get_observation_space`.

        Args:
            df (pd.DataFrame): network stats measurement

        Returns:
            spaces: observation spaces
        """

        rxPowerDbm = np.zeros((MAX_NUM_NODES_BSS0, MAX_NUM_NODES), dtype=np.float32)
        mcsIndex = np.zeros((MAX_NUM_NODES_BSS0, ), dtype=np.uint32)
        ulThpt = np.zeros((MAX_NUM_NODES, ), dtype=np.float32)
        vrDelay = np.zeros((1, ), dtype=np.float32)
        nodeLoc = np.zeros((MAX_NUM_NODES, 2), dtype=np.float32)

        for _, row in df.iterrows():
            ids = row['id']
            values = row['value']
            if row['source'] == 'Obss' and row['name'] == 'Cpp2Py::RxPowerDbmMatrix':
                for id, value in zip(ids, values):
                    # print("id=", id)
                    rxNum = id >> 5
                    txId = id & 0x1f
                    # print("rxId=", rxNum)
                    # print("txId=", txId)
                    rxPowerDbm[rxNum][txId] = value
                self.action_data_format = row
            elif row['source'] == 'Obss' and row['name'] == 'Cpp2Py::NodeX':
                nodeLoc[ids, 0] = values
            elif row['source'] == 'Obss' and row['name'] == 'Cpp2Py::NodeY':
                nodeLoc[ids, 1] = values
            elif row['source'] == 'Obss' and row['name'] == 'Cpp2Py::McsIndex':
                mcsIndex[ids] = values
            elif row['source'] == 'Obss' and row['name'] == 'Cpp2Py::UplinkThptMbps':
                ulThpt[ids] = values
            else:
                id = ids[0]
                value = values[0]
                if row['source'] == 'Obss' and row['name'] == 'Cpp2Py::AccessDelayMs':
                    vrDelay[0] = value  # id not used

        self.observation = (rxPowerDbm, mcsIndex, ulThpt, vrDelay, nodeLoc)
        # print("obs shape is", self.observation.shape)
        print('Observation --> ' + str(self.observation))

        return self.observation

    def get_policy(self, action):
        """Prepare policy for the nqos_split env.

        Args:
            action (spaces): action from the RL agent

        Returns:
            json: network policy
        """

        policy1 = json.loads(self.action_data_format.to_json())
        policy1['id'] = [0]
        policy1["name"] = "Py2Cpp::ObssPdNew"
        policy1["value"] = [action[0]]

        print('Action1 --> ' + str(policy1))
        self.action1_history.append(policy1["value"][0])

        policy2 = json.loads(self.action_data_format.to_json())
        policy2['id'] = [0]
        policy2["name"] = "Py2Cpp::TxPowerNew"
        policy2["value"] = [action[1]]

        print('Action2 --> ' + str(policy2))
        self.action2_history.append(policy2["value"][0])

        return [policy1, policy2]

    def get_reward(self, df):
        """Prepare reward for the nqos_split env.

        Args:
            df (pd.DataFrame): network stats

        Returns:
            spaces: reward spaces
        """

        ulThpt = self.observation[2]
        vrDelay = self.observation[3]
        ulThptTotal = np.sum(ulThpt)
        vrThpt = ulThpt[4]

        self.reward = REWARD_ALPHA * ulThptTotal + REWARD_BETA * (VR_DELAY_CONSTRAINT_MS - vrDelay) + REWARD_ETA * (vrThpt - VR_THPT_CONSTRAINT_MBPS)

        self.total_thpt_history.append(ulThptTotal)
        self.vr_thpt_history.append(vrThpt)

        print("ulThpt =", ulThpt)
        print("ulThptTotal =", ulThptTotal)
        print("vrThpt =", vrThpt)

        self.vr_delay_history.append(vrDelay)
        self.reward_history.append(self.reward)
        self.step_num_history.append(len(self.step_num_history) + 1)

        # visualization here
        self.visualize_network()

        return self.reward

    def visualize_network(self):
        if self.layout is None:
            return
        
        # visualize
        left = self.layout["left"]
        mixin_left = Panel(self.plotextMixin(self.make_plot))
        left.update(mixin_left)

        # show data
        right = self.layout["right"]
        right_table = self.make_table()
        right.update(right_table)

        if self.layout["main"].visible == False:
            self.layout["main"].visible = True

    def make_plot(self, width, height):
        nodeLoc = self.observation[4]
        plotext.clf()
        # TODO: get config from simulation
        x_h = [0, 50]
        y_h = [25, 25]
        x_v = [25, 25]
        y_v = [0, 50]
        plotext.plot(x_h, y_h, color="cyan", marker="_")
        plotext.plot(x_v, y_v, color="cyan", marker="|")
        for i in range(4 * (4 + 1)):    # total 20 nodes
            if i < 4:
                # Plot AP
                plotext.plot([nodeLoc[i][0]], [nodeLoc[i][1]], marker = "heart", color='red')
            else:
                # Plot STA
                if i == 4:
                    # VR STA
                    plotext.plot([nodeLoc[i][0]], [nodeLoc[i][1]], marker = "star", color='green')
                else:
                    plotext.plot([nodeLoc[i][0]], [nodeLoc[i][1]], marker = "star", color='white')
        plotext.xlim(0, 50)
        plotext.ylim(0, 50)
        plotext.plotsize(width, height)
        plotext.title("Scenario (♥ = AP, white * = normal STA, green * = VR STA)")
        plotext.theme('clear')
        return plotext.build()

    def make_table(self, n=20):
        # shows n most recent actions and rewards
        table = Table(title=f"Step history (showing {n} records)")
        actions_obsspd = np.full((n, ), np.nan)
        for i in range(min(n, len(self.action1_history))):
            actions_obsspd[i] = self.action1_history[-i-1]
        actions_txpower = np.full((n, ), np.nan)
        for i in range(min(n, len(self.action2_history))):
            actions_txpower[i] = self.action2_history[-i-1]
        rewards = np.full((n, ), np.nan)
        for i in range(min(n, len(self.reward_history))):
            rewards[i] = self.reward_history[-i-1]
        steps = np.full((n, ), np.nan)
        for i in range(min(n, len(self.step_num_history))):
            steps[i] = self.step_num_history[-i-1]
        total_thpts = np.full((n, ), np.nan)
        for i in range(min(n, len(self.total_thpt_history))):
            total_thpts[i] = self.total_thpt_history[-i-1]
        vr_thpts = np.full((n, ), np.nan)
        for i in range(min(n, len(self.vr_thpt_history))):
            vr_thpts[i] = self.vr_thpt_history[-i-1]
        vr_delays = np.full((n, ), np.nan)
        for i in range(min(n, len(self.vr_delay_history))):
            vr_delays[i] = self.vr_delay_history[-i-1]
        
        table.add_column("# timestep")
        table.add_column("total thpt (Mbps)")
        table.add_column("VR thpt (Mbps)")
        table.add_column("VR delay (ms)")
        table.add_column("reward")
        table.add_column("new OBSS_PD (dBm)")
        table.add_column("new TX Power (dBm)")

        for i in range(n):
            table.add_row("%.0f" % steps[i], "%.2f" % total_thpts[i], "%.2f" % vr_thpts[i], "%.2f" % vr_delays[i], "%.2f" % rewards[i], "%.0f" % actions_obsspd[i], "%.0f" % actions_txpower[i])
        
        panel = Panel(table)
        return panel

