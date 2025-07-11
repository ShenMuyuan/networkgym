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
import plotext as plt
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

    def get_action_space(self):
        """Get action space for the nqos_split env.

        Returns:
            spaces: action spaces
        """
        return spaces.MultiDiscrete(np.array([19]), start=np.array([2]), seed=42)

    #consistent with the get_observation function.
    def get_observation_space(self):
        """Get the observation space for nqos_split env.

        Returns:
            spaces: observation spaces
        """

        return spaces.MultiDiscrete(np.array([[10], [10]]), start=np.array([[1], [1]]), seed=42)
    
    def get_observation(self, df):
        """Prepare observation for nqos_split env.

        This function should return the same number of features defined in the :meth:`get_observation_space`.

        Args:
            df (pd.DataFrame): network stats measurement

        Returns:
            spaces: observation spaces
        """

        a_value = -1
        b_value = -1

        for _, row in df.iterrows():
            if row['source'] == 'calculator' and row['name'] == 'addend::a':
                a_value = row['value'][0]
                self.action_data_format = row
            if row['source'] == 'calculator' and row['name'] == 'addend::b':
                b_value = row['value'][0]

        print("a_value is", a_value)
        print("b_value is", b_value)

        observation = np.vstack([a_value, b_value])
        print("obs shape is", observation.shape)
        print('Observation --> ' + str(observation))
        return observation

    def get_policy(self, action):
        """Prepare policy for the nqos_split env.

        Args:
            action (spaces): action from the RL agent

        Returns:
            json: network policy
        """

        policy = json.loads(self.action_data_format.to_json())
        policy["name"] = "sum"
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

        a_value = -1
        b_value = -1

        for _, row in df.iterrows():
            if row['source'] == 'calculator' and row['name'] == 'addend::a':
                a_value = row['value'][0]
            if row['source'] == 'calculator' and row['name'] == 'addend::b':
                b_value = row['value'][0]

        # difference of the two addends (actually this value is not used by the algorithm)
        self.reward = a_value - b_value
        return self.reward
