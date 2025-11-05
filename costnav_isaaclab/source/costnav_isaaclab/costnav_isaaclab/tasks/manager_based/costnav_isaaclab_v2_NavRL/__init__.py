# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

# Register custom network builders for RL-Games (needed for mix_input_actor_critic)
from .rl_games_network import register_mix_input_network

register_mix_input_network()

##
# Register Gym environments.
##


gym.register(
    id="Template-Costnav-Isaaclab-v2-NavRL",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.costnav_isaaclab_env_cfg:CostnavIsaaclabEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_coco_train_mini.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:PPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
    },
)
