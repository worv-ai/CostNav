#!/usr/bin/env python3
"""Generate VS Code settings with Isaac Lab + CostNav search paths."""

from __future__ import annotations

import argparse
import os
import sys
import textwrap
from pathlib import Path
from typing import Iterable, List

VSCODE_TEMPLATE = """
{
    "editor.rulers": [79],
    "python.languageServer": "Pylance",
    "python.jediEnabled": false,
    "python.defaultInterpreterPath": "PYTHON.DEFAULTINTERPRETERPATH",
    "python.analysis.extraPaths": [
PYTHON.ANALYSIS.EXTRAPATHS
    ],
    "python.autoComplete.extraPaths": [
PYTHON.AUTOCOMPLETE.EXTRAPATHS
    ],
    "python.envFile": "${workspaceFolder}/.vscode/.python.env",
    "terminal.integrated.env.linux": {
        "ISAAC_PATH": "ISAAC.PATH",
        "CARB_APP_PATH": "CARB.APP.PATH",
        "EXP_PATH": "EXP.PATH",
        "PYTHONPATH": "PYTHON.PATH.ENV"
    },
    "python.formatting.provider": "black",
    "python.formatting.blackArgs": ["--line-length", "79"],
    "python.linting.pylintEnabled": false,
    "python.linting.flake8Enabled": true
}
"""

PYTHON_ENV_TEMPLATE = """# Python environment variables for Isaac Sim and Omni Kit
# This file is sourced by VS Code's Python extension

# Isaac Sim paths
ISAAC_PATH=ISAAC.PATH
CARB_APP_PATH=CARB.APP.PATH
EXP_PATH=EXP.PATH

# Python path including all Isaac Sim, Omni Kit, and extension directories
# Omni Kit kernel and core extensions
PYTHONPATH=${ISAAC_PATH}/kit/kernel/py
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/kit/exts
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/kit/extscore
# Isaac Sim extensions
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/exts
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/extscache
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/extsDeprecated
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/extsUser
# Python site-packages
PYTHONPATH=${PYTHONPATH}:${ISAAC_PATH}/kit/python/lib/python3.11/site-packages

# Library paths for binary interfaces
LD_LIBRARY_PATH=${ISAAC_PATH}/kit/lib
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ISAAC_PATH}/kit/plugins
LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ISAAC_PATH}/kit/exts
"""


def _find_isaac_sim_path() -> Path | None:
    """Find Isaac Sim installation path from environment or common locations."""
    # Try environment variable first
    isaac_path = os.environ.get("ISAAC_PATH") or os.environ.get("ISAAC_SIM_PATH")
    if isaac_path and Path(isaac_path).exists():
        return Path(isaac_path)

    # Try common installation paths
    common_paths = [
        Path("/isaac-sim"),  # Docker container path
        Path.home() / ".local/share/ov/pkg/isaac-sim-*",
    ]

    for path_pattern in common_paths:
        if "*" in str(path_pattern):
            # Handle glob patterns
            import glob
            matches = glob.glob(str(path_pattern))
            if matches:
                return Path(sorted(matches)[-1])  # Use latest version
        elif path_pattern.exists():
            return path_pattern

    return None


def _gather_extra_paths(repo_root: Path, isaac_sim_path: Path) -> List[str]:
    """Gather all Python paths needed for Isaac Sim, Omni Kit, and CostNav.

    Instead of adding individual extension directories (which creates hundreds of paths),
    we add the parent directories that contain extensions.
    """
    paths: List[str] = []

    # Omni Kit paths (kernel and extension directories)
    kit_path = isaac_sim_path / "kit"
    if kit_path.exists():
        # Kernel Python bindings
        kernel_py = kit_path / "kernel" / "py"
        if kernel_py.is_dir():
            paths.append(str(kernel_py))

        # Omni Kit extension directories (add parent dirs, not individual extensions)
        for folder_name in ["exts", "extscore"]:
            folder = kit_path / folder_name
            if folder.is_dir():
                paths.append(str(folder))

    # Isaac Sim extension directories (add parent dirs, not individual extensions)
    for folder_name in ["exts", "extscache", "extsDeprecated", "extsUser"]:
        folder = isaac_sim_path / folder_name
        if folder.is_dir():
            paths.append(str(folder))

    # Python site-packages
    site_packages = isaac_sim_path / "kit" / "python" / "lib" / "python3.11" / "site-packages"
    if site_packages.is_dir():
        paths.append(str(site_packages))

        # Isaac Lab source if installed in site-packages
        isaaclab_source = site_packages / "isaaclab" / "source" / "isaaclab"
        if isaaclab_source.is_dir():
            paths.append(str(isaaclab_source))

    # CostNav project directories
    for local_folder in ["costnav_isaaclab", "costnav"]:
        candidate = repo_root / local_folder
        if candidate.is_dir():
            paths.append(str(candidate))

    return paths


def _render_settings(isaac_sim_path: Path, extra_paths: Iterable[str]) -> str:
    """Render VS Code settings.json content."""
    # Use python.sh wrapper instead of direct python3 binary
    interpreter = str(isaac_sim_path / "python.sh")

    # Format paths for JSON
    entries = [f'"{path}"' for path in extra_paths]
    joined = ",\n".join(entries)
    joined = textwrap.indent(joined, " " * 8)

    # Build PYTHONPATH for terminal environment
    pythonpath_parts = [
        f"${{env:ISAAC_PATH}}/kit/kernel/py",
        f"${{env:ISAAC_PATH}}/kit/exts",
        f"${{env:ISAAC_PATH}}/kit/extscore",
        f"${{env:ISAAC_PATH}}/exts",
        f"${{env:ISAAC_PATH}}/extscache",
        f"${{env:ISAAC_PATH}}/extsDeprecated",
        f"${{env:ISAAC_PATH}}/extsUser",
        f"${{env:ISAAC_PATH}}/kit/python/lib/python3.11/site-packages",
        "${{env:PYTHONPATH}}"
    ]
    pythonpath_env = ":".join(pythonpath_parts)

    # Replace placeholders in template
    settings = VSCODE_TEMPLATE
    settings = settings.replace("PYTHON.DEFAULTINTERPRETERPATH", interpreter)
    settings = settings.replace("PYTHON.ANALYSIS.EXTRAPATHS", joined)
    settings = settings.replace("PYTHON.AUTOCOMPLETE.EXTRAPATHS", joined)
    settings = settings.replace("ISAAC.PATH", str(isaac_sim_path))
    settings = settings.replace("CARB.APP.PATH", str(isaac_sim_path / "kit"))
    settings = settings.replace("EXP.PATH", str(isaac_sim_path / "apps"))
    settings = settings.replace("PYTHON.PATH.ENV", pythonpath_env)

    return settings


def _render_python_env(isaac_sim_path: Path) -> str:
    """Render .python.env content."""
    env_content = PYTHON_ENV_TEMPLATE
    env_content = env_content.replace("ISAAC.PATH", str(isaac_sim_path))
    env_content = env_content.replace("CARB.APP.PATH", str(isaac_sim_path / "kit"))
    env_content = env_content.replace("EXP.PATH", str(isaac_sim_path / "apps"))
    return env_content


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate VS Code settings for CostNav + Isaac Lab"
    )
    parser.add_argument(
        "--isaac-sim",
        type=str,
        help="Path to Isaac Sim installation (default: auto-detect from ISAAC_PATH or /isaac-sim)"
    )
    parser.add_argument(
        "--output",
        default=".vscode/settings.json",
        help="Path to write settings.json (default: .vscode/settings.json)"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Overwrite existing settings file"
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]

    # Find Isaac Sim installation
    if args.isaac_sim:
        isaac_sim_path = Path(args.isaac_sim)
        if not isaac_sim_path.exists():
            print(f"Error: Isaac Sim path does not exist: {isaac_sim_path}")
            sys.exit(1)
    else:
        isaac_sim_path = _find_isaac_sim_path()
        if not isaac_sim_path:
            print("Error: Unable to locate Isaac Sim installation.")
            print("Please specify --isaac-sim /path/to/isaac-sim")
            sys.exit(1)

    print(f"Using Isaac Sim installation: {isaac_sim_path}")

    # Generate settings.json
    output_path = Path(args.output)
    if output_path.exists() and not args.force:
        print(f"VS Code settings already exist at {output_path}. Use --force to overwrite.")
        return

    extra_paths = _gather_extra_paths(repo_root, isaac_sim_path)
    if not extra_paths:
        print("Warning: No extra paths found.")

    settings = _render_settings(isaac_sim_path, extra_paths)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(settings)
    print(f"✓ VS Code settings written to {output_path}")

    # Generate .python.env
    python_env_path = output_path.parent / ".python.env"
    python_env = _render_python_env(isaac_sim_path)
    python_env_path.write_text(python_env)
    print(f"✓ Python environment file written to {python_env_path}")

    print(f"\nFound {len(extra_paths)} Python paths:")
    for path in extra_paths:
        print(f"  - {path}")


if __name__ == "__main__":
    main()
