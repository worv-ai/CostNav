# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Python module serving as a project/extension template.
"""

from pathlib import Path
import sys

# Ensure the shared `src` directory is importable so that we can reuse
# refactored configs without duplicating code under this extension.
_REPO_ROOT = Path(__file__).resolve().parents[4]
_SRC_PATH = _REPO_ROOT / "src"
if _SRC_PATH.is_dir():
    src_str = str(_SRC_PATH)
    if src_str not in sys.path:
        sys.path.insert(0, src_str)

from .compat import ensure_gymnasium_compat, ensure_isaaclab_imports

ensure_gymnasium_compat()
ensure_isaaclab_imports()

# Register Gym environments.
from .tasks import *  # noqa: F403

# Register UI extensions.
from .ui_extension_example import *  # noqa: F403
