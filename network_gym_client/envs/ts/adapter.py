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

    def get_action_space(self):
        """Get action space for the nqos_split env.

        Returns:
            spaces: action spaces
        """
        return spaces.MultiDiscrete(np.array([12]), start=np.array([0]), seed=42)

    #consistent with the get_observation function.
    def get_observation_space(self):
        """Get the observation space for nqos_split env.

        Returns:
            spaces: observation spaces
        """

        return spaces.Box(low=0, high=10000, shape=(2, 1), dtype=np.uint32)
    
    def get_observation(self, df):
        """Prepare observation for nqos_split env.

        This function should return the same number of features defined in the :meth:`get_observation_space`.

        Args:
            df (pd.DataFrame): network stats measurement

        Returns:
            spaces: observation spaces
        """

        succ_value = -1
        fail_value = -1

        for _, row in df.iterrows():
            if row['source'] == 'TsRateControl' and row['name'] == 'meas::succ':
                succ_value = row['value'][0]
                self.action_data_format = row
            if row['source'] == 'TsRateControl' and row['name'] == 'meas::fail':
                fail_value = row['value'][0]

        print("succ_value is", succ_value)
        print("fail_value is", fail_value)

        self.observation = np.vstack([succ_value, fail_value])
        print("obs shape is", self.observation.shape)
        print('Observation --> ' + str(self.observation))

        # visualization of observation and (previous) action
        self.visualize_thompson_sampling()

        return self.observation

    def get_policy(self, action):
        """Prepare policy for the nqos_split env.

        Args:
            action (spaces): action from the RL agent

        Returns:
            json: network policy
        """

        self.mcs = action[0]

        policy = json.loads(self.action_data_format.to_json())
        policy["name"] = "mcsNew"
        policy["value"] = action.tolist()

        print('Action --> ' + str(policy))
        return policy

    def get_reward(self, df):
        """Prepare reward for the nqos_split env.

        Args:
            df (pd.DataFrame): network stats

        Returns:
            spaces: reward spaces
        """

        succ_value = -1
        fail_value = -1

        for _, row in df.iterrows():
            if row['source'] == 'TsRateControl' and row['name'] == 'meas::succ':
                succ_value = row['value'][0]
                self.action_data_format = row
            if row['source'] == 'TsRateControl' and row['name'] == 'meas::fail':
                fail_value = row['value'][0]

        # PER (actually this value is not used by the algorithm)
        self.reward = succ_value / (succ_value + fail_value)
        return self.reward

    def visualize_thompson_sampling(self):
        # visualize the two nodes, the link between them, success and failure counts
        if self.layout is None:
            return
        left = self.layout["left"]
        mixin_left = Panel(self.plotextMixin(self.make_plot))
        left.update(mixin_left)

        if self.layout["main"].visible == False:
            self.layout["main"].visible = True

    def make_plot(self, width, height):
        plotext.clf()
        plotext.plot([0],[0], marker = "heart", color='white')
        plotext.text("STA", x=.1, y=0, alignment='center', color=231, background="blue")
        plotext.plot([0],[1], marker = "star", color='white')
        plotext.text("AP", x=.1, y=1, alignment='center', color=231, background="blue")
        if self.mcs is not None:
            plotext.text("Action: MCS="+str(self.mcs), x=.1, y=.60, alignment='left', color="gray", background="green")
        plotext.text("Obs: Succ="+str(self.observation[0][0]), x=.1, y=.50, alignment='left', color="gray", background="red")
        plotext.text("Obs: Fail="+str(self.observation[1][0]), x=.1, y=.40, alignment='left', color="gray", background="red")
        plotext.xlim(0, 0.5)
        plotext.plotsize(width, height)
        plotext.title("Network graph")
        plotext.theme('clear')
        return plotext.build()
