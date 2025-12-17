# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to evaluate a trained RL agent from skrl and collect statistics."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Evaluate a trained RL agent from skrl.")
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
parser.add_argument(
    "--ml_framework",
    type=str,
    default="torch",
    choices=["torch", "jax", "jax-numpy"],
    help="The ML framework used for training the skrl agent.",
)
parser.add_argument(
    "--algorithm",
    type=str,
    default="PPO",
    choices=["AMP", "PPO", "IPPO", "MAPPO"],
    help="The RL algorithm used for training the skrl agent.",
)
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

import os
import random
import time
from collections import defaultdict

import costnav_isaaclab.tasks  # noqa: F401, E402
import gymnasium as gym
import isaaclab_tasks  # noqa: F401, E402
import numpy as np
import skrl
import torch
from costnav_isaaclab.env_helpers import (
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
from isaaclab.utils.dict import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint
from isaaclab_rl.skrl import SkrlVecEnvWrapper
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config
from packaging import version

# check for minimum supported skrl version
SKRL_VERSION = "1.4.3"
if version.parse(skrl.__version__) < version.parse(SKRL_VERSION):
    skrl.logger.error(
        f"Unsupported skrl version: {skrl.__version__}. "
        f"Install supported version using 'pip install skrl>={SKRL_VERSION}'"
    )
    exit()

if args_cli.ml_framework.startswith("torch"):
    from skrl.utils.runner.torch import Runner
elif args_cli.ml_framework.startswith("jax"):
    from skrl.utils.runner.jax import Runner

# config shortcuts
algorithm = args_cli.algorithm.lower()
agent_cfg_entry_point = "skrl_cfg_entry_point" if algorithm in ["ppo"] else f"skrl_{algorithm}_cfg_entry_point"


@hydra_task_config(args_cli.task, agent_cfg_entry_point)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, experiment_cfg: dict):
    """Evaluate skrl agent and collect statistics."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # configure the ML framework into the global skrl variable
    if args_cli.ml_framework.startswith("jax"):
        skrl.config.jax.backend = "jax" if args_cli.ml_framework == "jax" else "numpy"

    # randomly sample a seed if seed = -1
    if args_cli.seed == -1:
        args_cli.seed = random.randint(0, 10000)

    # set the agent and environment seed from command line
    experiment_cfg["seed"] = args_cli.seed if args_cli.seed is not None else experiment_cfg["seed"]
    env_cfg.seed = experiment_cfg["seed"]

    # specify directory for logging experiments (load checkpoint)
    log_root_path = os.path.join("logs", "skrl", experiment_cfg["agent"]["experiment"]["directory"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    # get checkpoint path
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("skrl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = os.path.abspath(args_cli.checkpoint)
    else:
        # specify name of checkpoint
        if args_cli.use_last_checkpoint:
            checkpoint_file = ".*"
        else:
            # this loads the best checkpoint (default behavior)
            checkpoint_file = "best_agent.pt"
        resume_path = get_checkpoint_path(
            log_root_path,
            run_dir=f".*_{algorithm}_{args_cli.ml_framework}",
            checkpoint=checkpoint_file,
            other_dirs=["checkpoints"],
        )
    log_dir = os.path.dirname(os.path.dirname(resume_path))

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv) and algorithm in ["ppo"]:
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "eval"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during evaluation.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for skrl
    env = SkrlVecEnvWrapper(env, ml_framework=args_cli.ml_framework)

    # find the underlying Isaac Lab env that exposes the scene and contact sensor
    base_env = get_env_with_scene(env)
    if base_env is None:
        print(
            "[INFO] Could not locate underlying Isaac Lab env with 'scene'; collision/work metrics will be disabled."
        )
    else:
        print("[INFO] Collision / work metrics enabled from contact_forces sensor (excluding wheels).")

    # configure and instantiate the skrl runner
    experiment_cfg["trainer"]["close_environment_at_exit"] = False
    experiment_cfg["agent"]["experiment"]["write_interval"] = 0  # don't log to TensorBoard
    experiment_cfg["agent"]["experiment"]["checkpoint_interval"] = 0  # don't generate checkpoints

    # Check if custom models are enabled
    custom_model_cfg = experiment_cfg.get("custom_model", {})
    use_custom_models = custom_model_cfg.get("enabled", False)

    if use_custom_models:
        # Import and instantiate custom models
        import importlib

        module_path = custom_model_cfg["module"]
        factory_name = custom_model_cfg["factory"]
        model_params = custom_model_cfg.get("params", {})

        print(f"[INFO] Using custom models from {module_path}.{factory_name}")

        # Import the factory function
        module = importlib.import_module(module_path)
        factory_fn = getattr(module, factory_name)

        # Create custom models
        models = factory_fn(
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=env.device,
            **model_params,
        )

        # Create agent manually with custom models
        from skrl.agents.torch.ppo import PPO
        from skrl.memories.torch import RandomMemory

        # Configure memory
        memory_cfg = experiment_cfg.get("memory", {})
        memory_size = memory_cfg.get("memory_size", -1)
        if memory_size == -1:
            memory_size = experiment_cfg["agent"]["rollouts"]
        memory = RandomMemory(memory_size=memory_size, num_envs=env.num_envs, device=env.device)

        # Configure PPO agent
        ppo_cfg = experiment_cfg["agent"].copy()
        ppo_cfg.pop("class", None)  # Remove class key

        # Handle learning rate scheduler
        lr_scheduler = ppo_cfg.pop("learning_rate_scheduler", None)
        lr_scheduler_kwargs = ppo_cfg.pop("learning_rate_scheduler_kwargs", {})
        if lr_scheduler == "KLAdaptiveLR":
            from skrl.resources.schedulers.torch import KLAdaptiveLR

            ppo_cfg["learning_rate_scheduler"] = KLAdaptiveLR
            ppo_cfg["learning_rate_scheduler_kwargs"] = lr_scheduler_kwargs or {}

        # Handle state/value preprocessors
        state_preprocessor = ppo_cfg.pop("state_preprocessor", None)
        ppo_cfg.pop("state_preprocessor_kwargs", None)  # Remove from config, we'll set our own
        if state_preprocessor == "RunningStandardScaler":
            from skrl.resources.preprocessors.torch import RunningStandardScaler

            ppo_cfg["state_preprocessor"] = RunningStandardScaler
            ppo_cfg["state_preprocessor_kwargs"] = {"size": env.observation_space, "device": env.device}

        value_preprocessor = ppo_cfg.pop("value_preprocessor", None)
        ppo_cfg.pop("value_preprocessor_kwargs", None)  # Remove from config, we'll set our own
        if value_preprocessor == "RunningStandardScaler":
            from skrl.resources.preprocessors.torch import RunningStandardScaler

            ppo_cfg["value_preprocessor"] = RunningStandardScaler
            ppo_cfg["value_preprocessor_kwargs"] = {"size": 1, "device": env.device}

        # Create PPO agent
        agent = PPO(
            models=models,
            memory=memory,
            observation_space=env.observation_space,
            action_space=env.action_space,
            device=env.device,
            cfg=ppo_cfg,
        )

        # Create a simple runner-like object for compatibility
        class CustomRunner:
            def __init__(self, agent):
                self.agent = agent

        runner = CustomRunner(agent)
    else:
        # Use standard Runner with model instantiators
        runner = Runner(env, experiment_cfg)

    print(f"[INFO] Loading model checkpoint from: {resume_path}")
    runner.agent.load(resume_path)
    # set agent to evaluation mode
    runner.agent.set_running_mode("eval")

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
    episode_terminations = []

    num_envs = env.unwrapped.num_envs
    device = env.unwrapped.device
    current_episode_rewards = torch.zeros(num_envs, device=device)
    current_episode_lengths = torch.zeros(num_envs, device=device, dtype=torch.int)

    # Collision impulse metrics (per environment, per episode)
    current_episode_collision_steps = torch.zeros(num_envs, device=device, dtype=torch.int)
    current_episode_collision_impulse = torch.zeros(num_envs, device=device)
    current_episode_collision_any = torch.zeros(num_envs, device=device, dtype=torch.bool)

    collision_dt = None

    # Navigation energy / power metrics (per environment, per episode)
    current_episode_nav_energy = torch.zeros(num_envs, device=device)
    current_episode_nav_power_max = torch.zeros(num_envs, device=device)

    nav_dt = None

    # Get available termination terms from the environment
    termination_term_names = []
    if hasattr(env.unwrapped, "termination_manager"):
        termination_term_names = list(env.unwrapped.termination_manager._term_names)
        print(f"[INFO] Tracking termination terms: {termination_term_names}")

    completed_episodes = 0
    total_steps = 0

    # reset environment
    obs, _ = env.reset()

    start_time = time.time()

    # simulate environment
    while simulation_app.is_running() and completed_episodes < args_cli.num_episodes:
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping - get deterministic actions
            outputs = runner.agent.act(obs, timestep=0, timesteps=0)
            # extract deterministic actions (mean_actions for stochastic policies)
            if hasattr(env, "possible_agents"):
                actions = {a: outputs[-1][a].get("mean_actions", outputs[0][a]) for a in env.possible_agents}
            else:
                actions = outputs[-1].get("mean_actions", outputs[0])
            # env stepping
            obs, rewards, terminated, truncated, infos = env.step(actions)
            dones = terminated | truncated

            # Track rewards and lengths
            current_episode_rewards += rewards.squeeze()
            current_episode_lengths += 1
            total_steps += num_envs

            # Collision / work metrics from contact sensor (if available)
            if base_env is not None:
                metrics = compute_contact_impulse_metrics(base_env)
                if metrics is not None and metrics["max_force"] is not None:
                    max_force = metrics["max_force"]
                    collision_mask = max_force > 0.0

                    if collision_dt is None:
                        collision_dt = metrics["dt"]

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

                # Navigation energy / power metrics
                nav_metrics = compute_navigation_energy_step(base_env)
                if nav_metrics is not None and nav_metrics.get("power") is not None:
                    if nav_dt is None:
                        nav_dt = nav_metrics.get("dt")

                    step_power = nav_metrics["power"].to(current_episode_nav_energy.device)

                    if nav_dt is not None:
                        step_energy = step_power * float(nav_dt)
                        current_episode_nav_energy += step_energy

                    current_episode_nav_power_max = torch.max(current_episode_nav_power_max, step_power)

            # Check for completed episodes
            if dones.any():
                done_indices = torch.where(dones.squeeze())[0]

                for idx in done_indices:
                    # Episode-level reward/length
                    episode_rewards.append(current_episode_rewards[idx].item())
                    episode_lengths.append(current_episode_lengths[idx].item())

                    # Track termination reason
                    termination_reason = "unknown"
                    if hasattr(env.unwrapped, "termination_manager") and termination_term_names:
                        for term_name in termination_term_names:
                            term_value = env.unwrapped.termination_manager.get_term(term_name)
                            if term_value[idx]:
                                termination_reason = term_name
                                termination_counts[term_name] += 1
                                break

                    episode_terminations.append(termination_reason)

                    # Collect additional info from infos dict
                    if isinstance(infos, dict):
                        for key, value in infos.items():
                            if isinstance(value, torch.Tensor) and value.numel() == num_envs:
                                episode_data[key].append(value[idx].item())

                    # Collision / work metrics for this finished episode
                    if base_env is not None:
                        episode_collision_steps = current_episode_collision_steps[idx].item()
                        episode_collision_impulse = current_episode_collision_impulse[idx].item()

                        episode_data["collision_steps"].append(episode_collision_steps)
                        episode_data["collision_impulse"].append(episode_collision_impulse)

                        # Navigation energy / power metrics
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
