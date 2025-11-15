# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Compatibility helpers for external libraries."""

from __future__ import annotations

import sys
from pathlib import Path
from types import ModuleType


def ensure_gymnasium_compat() -> None:
    """Alias Gymnasium as Gym so downstream deps don't import the deprecated Gym package.

    When RL-Games (or other third-party libraries) import :mod:`gym` they trigger Gym's deprecation
    banner and, in newer Python/Numpy stacks, runtime issues. Isaac Lab already standardizes on
    Gymnasium, so we inject it under the ``gym`` namespace before those libraries import it.
    """

    if "gym" in sys.modules:
        # Gym (or an alias) already imported, nothing to do.
        return

    import gymnasium as _gymnasium

    def _alias(name: str, module: ModuleType) -> None:
        if name not in sys.modules:
            sys.modules[name] = module

    _alias("gym", _gymnasium)
    _alias("gym.spaces", _gymnasium.spaces)
    _alias("gym.envs", _gymnasium.envs)
    _alias("gym.vector", _gymnasium.vector)
    _alias("gym.wrappers", _gymnasium.wrappers)


def ensure_isaaclab_imports() -> None:
    """Expose the packaged Isaac Lab sources under the canonical module names.

    The wheel layout ships the actual python sources under ``isaaclab/source`` while the
    installed package only bootstraps the application launcher. Downstream extensions expect
    modules like ``isaaclab.assets`` / ``isaaclab_rl`` to import directly, so we extend the
    module search path at runtime to include those sources.
    """

    try:
        import isaaclab as _isaaclab_pkg
    except ImportError:
        return

    isaaclab_root = Path(getattr(_isaaclab_pkg, "__file__", "")).resolve().parent
    source_root = isaaclab_root / "source"
    if not source_root.exists():
        return

    source_str = str(source_root)
    if source_str not in sys.path:
        sys.path.insert(0, source_str)

    isaaclab_core = source_root / "isaaclab"
    isaaclab_core_pkg = isaaclab_core / "isaaclab"
    if isaaclab_core_pkg.exists():
        pkg_path = getattr(_isaaclab_pkg, "__path__", None)
        if pkg_path is not None and str(isaaclab_core_pkg) not in pkg_path:
            pkg_path.append(str(isaaclab_core_pkg))

    for nested_pkg in ("isaaclab_tasks", "isaaclab_rl", "isaaclab_assets", "isaaclab_mimic"):
        parent_dir = source_root / nested_pkg
        pkg_dir = parent_dir / nested_pkg
        if parent_dir.exists() and pkg_dir.exists():
            parent_str = str(parent_dir)
            if parent_str not in sys.path:
                sys.path.insert(0, parent_str)
