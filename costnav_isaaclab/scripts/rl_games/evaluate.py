# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to evaluate a trained RL agent from RL-Games and collect statistics."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Evaluate a trained RL agent from RL-Games.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during evaluation.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric",
    action="store_true",
    default=False,
    help="Disable fabric and use USD I/O operations.",
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--checkpoint", type=str, default=None, help="Path to model checkpoint.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument(
    "--use_last_checkpoint",
    action="store_true",
    help="When no checkpoint provided, use the last saved model. Otherwise use the best saved model.",
)
parser.add_argument("--num_episodes", type=int, default=100, help="Number of episodes to evaluate.")
parser.add_argument("--output_file", type=str, default=None, help="Path to save evaluation results (CSV).")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""


import math
import os
import random
import time
from collections import defaultdict

import costnav_isaaclab.tasks  # noqa: F401, E402
import gymnasium as gym
import isaaclab_tasks  # noqa: F401, E402
import numpy as np
import torch
from costnav_isaaclab.rl_games_helpers import (
    compute_contact_impulse_metrics,
    compute_navigation_energy_step,
    get_env_with_scene,
)
from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint
from isaaclab_rl.rl_games import RlGamesGpuEnv, RlGamesVecEnvWrapper
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config
from rl_games.common import env_configurations, vecenv
from rl_games.common.player import BasePlayer
from rl_games.torch_runner import Runner


@hydra_task_config(args_cli.task, "rl_games_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: dict):
    """Evaluate RL-Games agent and collect statistics."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # randomly sample a seed if seed = -1
    if args_cli.seed == -1:
        args_cli.seed = random.randint(0, 10000)

    agent_cfg["params"]["seed"] = args_cli.seed if args_cli.seed is not None else agent_cfg["params"]["seed"]
    # set the environment seed (after multi-gpu config for updated rank from agent seed)
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg["params"]["seed"]

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rl_games", agent_cfg["params"]["config"]["name"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    # find checkpoint
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rl_games", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint is None:
        # specify directory for logging runs
        run_dir = agent_cfg["params"]["config"].get("full_experiment_name", ".*")
        # specify name of checkpoint
        if args_cli.use_last_checkpoint:
            checkpoint_file = ".*"
        else:
            # this loads the best checkpoint
            checkpoint_file = f"{agent_cfg['params']['config']['name']}.pth"
        # get path to previous checkpoint
        resume_path = get_checkpoint_path(log_root_path, run_dir, checkpoint_file, other_dirs=["nn"])
    else:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    log_dir = os.path.dirname(os.path.dirname(resume_path))

    # wrap around environment for rl-games
    rl_device = agent_cfg["params"]["config"]["device"]
    clip_obs = agent_cfg["params"]["env"].get("clip_observations", math.inf)
    clip_actions = agent_cfg["params"]["env"].get("clip_actions", math.inf)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_root_path, log_dir, "videos", "eval"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during evaluation.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rl-games
    env = RlGamesVecEnvWrapper(env, rl_device, clip_obs, clip_actions)

    # find the underlying Isaac Lab env that exposes the scene and contact sensor
    base_env = get_env_with_scene(env)
    if base_env is None:
        print(
            "[INFO] Could not locate underlying Isaac Lab env with 'scene'; collision/work metrics will be disabled."
        )
    else:
        print("[INFO] Collision / work metrics enabled from contact_forces sensor (excluding wheels).")

    # register the environment to rl-games registry
    # note: in agents configuration: environment name must be "rlgpu"
    vecenv.register(
        "IsaacRlgWrapper",
        lambda config_name, num_actors, **kwargs: RlGamesGpuEnv(config_name, num_actors, **kwargs),
    )
    env_configurations.register("rlgpu", {"vecenv_type": "IsaacRlgWrapper", "env_creator": lambda **kwargs: env})

    # load previously trained model
    agent_cfg["params"]["load_checkpoint"] = True
    agent_cfg["params"]["load_path"] = resume_path
    print(f"[INFO]: Loading model checkpoint from: {agent_cfg['params']['load_path']}")

    # set number of actors into agent config
    agent_cfg["params"]["config"]["num_actors"] = env.unwrapped.num_envs
    # create runner from rl-games
    runner = Runner()
    runner.load(agent_cfg)
    # obtain the agent from the runner
    agent: BasePlayer = runner.create_player()
    agent.restore(resume_path)
    agent.reset()

    print(f"\n{'=' * 80}")
    print("STARTING EVALUATION")
    print(f"{'=' * 80}")
    print(f"Task: {args_cli.task}")
    print(f"Checkpoint: {resume_path}")
    print(f"Number of environments: {env.unwrapped.num_envs}")
    print(f"Target episodes: {args_cli.num_episodes}")
    print(f"{'=' * 80}\n")

    # Statistics tracking
    episode_rewards = []
    episode_lengths = []
    episode_data = defaultdict(list)

    # Termination tracking
    termination_counts = defaultdict(int)
    episode_terminations = []  # Store termination reason for each episode

    num_envs = env.unwrapped.num_envs
    current_episode_rewards = torch.zeros(num_envs, device=env.unwrapped.device)
    current_episode_lengths = torch.zeros(num_envs, device=env.unwrapped.device, dtype=torch.int)

    # Collision impulse metrics (per environment, per episode)
    current_episode_collision_steps = torch.zeros(num_envs, device=env.unwrapped.device, dtype=torch.int)
    current_episode_collision_impulse = torch.zeros(num_envs, device=env.unwrapped.device)
    current_episode_collision_any = torch.zeros(num_envs, device=env.unwrapped.device, dtype=torch.bool)

    collision_dt = None  # Filled lazily from the contact sensor metrics

    # Navigation energy / power metrics (per environment, per episode)
    current_episode_nav_energy = torch.zeros(num_envs, device=env.unwrapped.device)
    current_episode_nav_power_max = torch.zeros(num_envs, device=env.unwrapped.device)

    nav_dt = None  # Filled lazily from the navigation energy helper

    # Get available termination terms from the environment
    termination_term_names = []
    if hasattr(env.unwrapped, "termination_manager"):
        termination_term_names = list(env.unwrapped.termination_manager._term_names)
        print(f"[INFO] Tracking termination terms: {termination_term_names}")

    completed_episodes = 0
    total_steps = 0

    # reset environment
    obs = env.reset()
    if isinstance(obs, dict):
        obs = obs["obs"]

    # required: enables the flag for batched observations
    _ = agent.get_batch_size(obs, 1)
    # initialize RNN states if used
    if agent.is_rnn:
        agent.init_rnn()

    start_time = time.time()

    # simulate environment
    while simulation_app.is_running() and completed_episodes < args_cli.num_episodes:
        # run everything in inference mode
        with torch.inference_mode():
            # convert obs to agent format
            obs = agent.obs_to_torch(obs)
            # agent stepping
            actions = agent.get_action(obs, is_deterministic=agent.is_deterministic)
            # env stepping
            obs, rewards, dones, infos = env.step(actions)

            # Track rewards and lengths
            current_episode_rewards += rewards
            current_episode_lengths += 1
            total_steps += num_envs

            # Collision / work metrics from contact sensor (if available)
            if base_env is not None:
                metrics = compute_contact_impulse_metrics(base_env)
                if metrics is not None and metrics["max_force"] is not None:
                    max_force = metrics["max_force"]  # (num_envs,)
                    collision_mask = max_force > 0.0

                    # lazily capture dt from metrics
                    if collision_dt is None:
                        collision_dt = metrics["dt"]

                    # Approximate per-step collision impulse (N·s) and accumulate over collision steps.
                    step_impulse = metrics.get("total_impulse_dt")
                    if step_impulse is None:
                        step_impulse = metrics.get("max_impulse_dt")
                    if step_impulse is not None:
                        step_impulse = step_impulse.to(current_episode_collision_impulse.device)
                        current_episode_collision_impulse += torch.where(
                            collision_mask,
                            step_impulse,
                            torch.zeros_like(step_impulse),
                        )

                    current_episode_collision_steps += collision_mask.to(current_episode_collision_steps.dtype)
                    current_episode_collision_any |= collision_mask

                # Navigation energy / power metrics based on m * g * v
                nav_metrics = compute_navigation_energy_step(base_env)
                if nav_metrics is not None and nav_metrics.get("power") is not None:
                    if nav_dt is None:
                        nav_dt = nav_metrics.get("dt")

                    step_power = nav_metrics["power"].to(current_episode_nav_energy.device)

                    # Integrate power over time to approximate energy consumption.
                    if nav_dt is not None:
                        step_energy = step_power * float(nav_dt)
                        current_episode_nav_energy += step_energy

                    current_episode_nav_power_max = torch.max(current_episode_nav_power_max, step_power)

            # Check for completed episodes
            if len(dones) > 0:
                done_indices = torch.where(dones)[0]

                for idx in done_indices:
                    # Episode-level reward/length
                    episode_rewards.append(current_episode_rewards[idx].item())
                    episode_lengths.append(current_episode_lengths[idx].item())

                    # Track termination reason
                    termination_reason = "unknown"
                    if hasattr(env.unwrapped, "termination_manager") and termination_term_names:
                        # Check which termination term triggered
                        for term_name in termination_term_names:
                            term_value = env.unwrapped.termination_manager.get_term(term_name)
                            if term_value[idx]:
                                termination_reason = term_name
                                termination_counts[term_name] += 1
                                break

                    episode_terminations.append(termination_reason)

                    # Collect additional info from infos dict (if provided by env)
                    if isinstance(infos, dict):
                        for key, value in infos.items():
                            if isinstance(value, torch.Tensor) and value.numel() == num_envs:
                                episode_data[key].append(value[idx].item())

                    # Collision / work metrics for this finished episode
                    if base_env is not None:
                        # Number of steps with non-zero collision force
                        episode_collision_steps = current_episode_collision_steps[idx].item()
                        episode_collision_impulse = current_episode_collision_impulse[idx].item()

                        episode_data["collision_steps"].append(episode_collision_steps)
                        episode_data["collision_impulse"].append(episode_collision_impulse)

                        # Navigation energy / power metrics (m * g * v) for this finished episode
                        episode_nav_energy = current_episode_nav_energy[idx].item()
                        episode_nav_energy_kwh = episode_nav_energy / 3_600_000.0
                        episode_nav_power_max = current_episode_nav_power_max[idx].item()

                        episode_data["nav_energy_mgv"].append(episode_nav_energy)
                        episode_data["nav_energy_kwh_mgv"].append(episode_nav_energy_kwh)
                        episode_data["nav_power_max_mgv"].append(episode_nav_power_max)

                        if nav_dt is not None and current_episode_lengths[idx] > 0:
                            episode_time_s_nav = float(current_episode_lengths[idx].item()) * float(nav_dt)
                            if episode_time_s_nav > 0.0:
                                episode_nav_power_avg = episode_nav_energy / episode_time_s_nav
                            else:
                                episode_nav_power_avg = 0.0
                        else:
                            episode_nav_power_avg = 0.0

                        episode_data["nav_power_avg_mgv"].append(episode_nav_power_avg)

                    completed_episodes += 1

                    # Print progress
                    if completed_episodes % 10 == 0:
                        elapsed = time.time() - start_time
                        print(
                            f"[{completed_episodes}/{args_cli.num_episodes}] "
                            f"Avg Reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f} | "
                            f"Avg Length: {np.mean(episode_lengths):.1f} | "
                            f"Time: {elapsed:.1f}s"
                        )

                    # Reset tracking for this environment
                    current_episode_rewards[idx] = 0
                    current_episode_lengths[idx] = 0
                    current_episode_collision_steps[idx] = 0
                    current_episode_collision_impulse[idx] = 0.0
                    current_episode_collision_any[idx] = False
                    current_episode_nav_energy[idx] = 0.0
                    current_episode_nav_power_max[idx] = 0.0

                # reset rnn state for terminated episodes
                if agent.is_rnn and agent.states is not None:
                    for s in agent.states:
                        s[:, dones, :] = 0.0

                    episode_terminations.append(termination_reason)

                    # Collect additional info if available
                    if isinstance(infos, dict):
                        for key, value in infos.items():
                            if isinstance(value, torch.Tensor) and value.numel() == num_envs:
                                episode_data[key].append(value[idx].item())

                    completed_episodes += 1

                    # Print progress
                    if completed_episodes % 10 == 0:
                        elapsed = time.time() - start_time
                        print(
                            f"[{completed_episodes}/{args_cli.num_episodes}] "
                            f"Avg Reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f} | "
                            f"Avg Length: {np.mean(episode_lengths):.1f} | "
                            f"Time: {elapsed:.1f}s"
                        )

                    # Reset tracking for this environment
                    current_episode_rewards[idx] = 0
                    current_episode_lengths[idx] = 0

                # reset rnn state for terminated episodes
                if agent.is_rnn and agent.states is not None:
                    for s in agent.states:
                        s[:, dones, :] = 0.0

    # Calculate final statistics
    elapsed_time = time.time() - start_time

    print(f"\n{'=' * 80}")
    print("EVALUATION COMPLETE")
    print(f"{'=' * 80}")
    print(f"Total episodes: {completed_episodes}")
    print(f"Total steps: {total_steps}")
    print(f"Total time: {elapsed_time:.2f}s")
    print(f"Steps per second: {total_steps / elapsed_time:.1f}")
    print("\nEpisode Rewards:")
    print(f"  Mean: {np.mean(episode_rewards):.4f}")
    print(f"  Std:  {np.std(episode_rewards):.4f}")
    print(f"  Min:  {np.min(episode_rewards):.4f}")
    print(f"  Max:  {np.max(episode_rewards):.4f}")
    print("\nEpisode Lengths:")
    print(f"  Mean: {np.mean(episode_lengths):.2f}")
    print(f"  Std:  {np.std(episode_lengths):.2f}")
    print(f"  Min:  {int(np.min(episode_lengths))}")
    print(f"  Max:  {int(np.max(episode_lengths))}")

    # Print termination statistics
    if termination_counts:
        print("\nEpisode Terminations:")
        total_episodes = sum(termination_counts.values())

        # Sort by count (descending)
        sorted_terms = sorted(termination_counts.items(), key=lambda x: x[1], reverse=True)

        for term_name, count in sorted_terms:
            percentage = (count / total_episodes * 100) if total_episodes > 0 else 0
            print(f"  {term_name:20s}: {count:4d} ({percentage:5.1f}%)")

    # Print additional metrics if available
    if episode_data:
        print("\nAdditional Metrics:")
        for key, values in episode_data.items():
            if len(values) > 0:
                print(f"  {key}:")
                print(f"    Mean: {np.mean(values):.4f}")
                print(f"    Std:  {np.std(values):.4f}")

    print(f"{'=' * 80}\n")

    # Save results to file if requested
    if args_cli.output_file:
        import csv

        output_path = args_cli.output_file
        os.makedirs(os.path.dirname(output_path) if os.path.dirname(output_path) else ".", exist_ok=True)

        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            # Write header
            header = ["episode", "reward", "length", "termination"] + list(episode_data.keys())
            writer.writerow(header)

            # Write data
            for i in range(len(episode_rewards)):
                row = [
                    i,
                    episode_rewards[i],
                    episode_lengths[i],
                    episode_terminations[i] if i < len(episode_terminations) else "unknown",
                ]
                for key in episode_data.keys():
                    row.append(episode_data[key][i] if i < len(episode_data[key]) else "")
                writer.writerow(row)

        print(f"[INFO] Results saved to: {output_path}")

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
