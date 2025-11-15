# Helper utilities shared between RL-Games play/evaluate scripts.

from __future__ import annotations

from typing import Any, Dict, Optional

import torch


def get_env_with_scene(env: Any) -> Optional[Any]:
    """Return the underlying Isaac Lab environment that has a ``scene`` attribute.

    This walks through ``.unwrapped`` until an environment with ``scene`` is found
    or there is nothing left to unwrap.
    """
    base = getattr(env, "unwrapped", env)
    visited: set[int] = set()
    while True:
        if hasattr(base, "scene"):
            return base
        if not hasattr(base, "unwrapped") or base is base.unwrapped:
            return None
        if id(base) in visited:
            # Safety guard against potential cycles.
            return None
        visited.add(id(base))
        base = base.unwrapped


def compute_contact_impulse_metrics(
    env_with_scene: Any, sensor_name: str = "contact_forces"
) -> Optional[Dict[str, Any]]:
    """Compute contact force and impulse-by-dt metrics from the ContactSensor.

    Mirrors the logic used in the task's ``print_contact_impulses`` debug reward:
    - Reads ``sensor.data.net_forces_w`` (shape: num_envs x num_bodies x 3).
    - Excludes wheel links from the aggregation.
    - Approximates per-step impulse as ``force * dt``.

    Returns a dict with per-env tensors (force and impulse) or ``None`` if the
    contact sensor is not available.
    """
    try:
        scene = env_with_scene.scene
    except AttributeError:
        return None

    try:
        sensor = scene[sensor_name]
    except Exception:
        return None

    if not hasattr(sensor, "data") or not hasattr(sensor.data, "net_forces_w"):
        return None

    net_forces = sensor.data.net_forces_w
    if net_forces is None:
        return None

    # Expected shape is (num_envs, num_bodies, 3).
    if net_forces.ndim != 3:
        try:
            num_envs = env_with_scene.num_envs
            num_bodies = net_forces.shape[0] // num_envs
            net_forces = net_forces.view(num_envs, num_bodies, 3)
        except Exception:
            return None

    # Per-body contact magnitude (||force|| in N).
    magnitudes = torch.norm(net_forces, dim=-1)

    # Exclude wheel links (by suffix).
    except_suffixes = (
        "front_left_wheel_link",
        "front_right_wheel_link",
        "rear_left_wheel_link",
        "rear_right_wheel_link",
    )
    body_names = getattr(sensor, "body_names", None)
    if body_names is not None:
        try:
            exclude_indices = [
                idx
                for idx, name in enumerate(body_names)
                if any(str(name).endswith(suffix) for suffix in except_suffixes)
            ]
            if exclude_indices:
                magnitudes[:, exclude_indices] = 0.0
        except Exception:
            # If exclusion fails, continue with unfiltered magnitudes.
            pass

    total_force = magnitudes.sum(dim=1)
    max_force = magnitudes.max(dim=1).values

    # Determine dt for impulse approximation.
    dt = getattr(env_with_scene, "step_dt", None)
    if dt is None:
        try:
            dt = float(env_with_scene.cfg.sim.dt) * float(env_with_scene.cfg.decimation)
        except Exception:
            dt = None

    if dt is not None:
        impulse_mags = magnitudes * float(dt)
        total_impulse = impulse_mags.sum(dim=1)
        max_impulse = impulse_mags.max(dim=1).values
    else:
        total_impulse = None
        max_impulse = None

    return {
        "dt": dt,
        "total_force": total_force,
        "max_force": max_force,
        "total_impulse_dt": total_impulse,
        "max_impulse_dt": max_impulse,
    }


def compute_navigation_energy_step(
    env_with_scene: Any, gravity: float = 9.81
) -> Optional[Dict[str, Any]]:
    """Compute per-env navigation energy/power proxy based on ``m * g * v``.

    - ``m``: robot mass (sum of body/link masses) if available; otherwise 1.0.
    - ``g``: gravitational acceleration (default 9.81 m/s^2).
    - ``v``: planar base speed computed from the root linear velocity.

    Returns a dict with keys ``dt``, ``speed``, ``power``, and ``mass``, or
    ``None`` if the required data is not available.
    """
    # Locate the robot articulation inside the scene.
    try:
        scene = env_with_scene.scene
    except AttributeError:
        return None

    robot = None
    for key in ("robot", "Robot"):
        try:
            robot = scene[key]
            break
        except Exception:
            continue
    if robot is None or not hasattr(robot, "data"):
        return None

    data = robot.data

    # Prefer base-frame linear velocity if available, otherwise fall back to
    # world-frame root state.
    if hasattr(data, "root_lin_vel_b"):
        root_vel = data.root_lin_vel_b
    elif hasattr(data, "root_state_w"):
        rs = data.root_state_w
        root_vel = rs[..., 7:10]
    else:
        return None

    if root_vel is None:
        return None

    # Use planar (x, y) speed as navigation speed.
    speed = torch.norm(root_vel[..., :2], dim=-1)

    # Determine dt.
    dt = getattr(env_with_scene, "step_dt", None)
    if dt is None:
        try:
            dt = float(env_with_scene.cfg.sim.dt) * float(env_with_scene.cfg.decimation)
        except Exception:
            dt = None

    # Estimate mass per environment.
    num_envs = getattr(env_with_scene, "num_envs", None)
    device = getattr(env_with_scene, "device", speed.device)

    mass_per_env = None
    for attr in ("body_mass", "mass"):
        if hasattr(data, attr) and num_envs is not None:
            masses = getattr(data, attr)
            if isinstance(masses, torch.Tensor) and masses.numel() > 0:
                try:
                    masses = masses.view(num_envs, -1)
                    mass_per_env = masses.sum(dim=1)
                    break
                except Exception:
                    continue

    if mass_per_env is None:
        # Fallback: assume a fixed mass (50 kg) for all environments. This preserves the
        # shape and relative scaling of the metric even if exact masses are
        # unavailable and approximates the COCO robot mass for evaluation.
        if num_envs is None:
            return None
        mass_per_env = torch.full((num_envs,), 50.0, device=device)

    power = mass_per_env * float(gravity) * speed

    return {"dt": dt, "speed": speed, "power": power, "mass": mass_per_env}
