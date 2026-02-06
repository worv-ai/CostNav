#!/usr/bin/env python3
"""
Update Omniverse asset paths in the codebase.

This script replaces the internal Omniverse server URL with a local server URL
or local file paths for users who have downloaded the assets.

Usage:
    # Replace with localhost (for local Nucleus server):
    python scripts/assets/update_asset_paths.py --target localhost

    # Replace with local file paths:
    python scripts/assets/update_asset_paths.py --target local

    # Revert to original internal server:
    python scripts/assets/update_asset_paths.py --target internal

    # Dry run (show changes without applying):
    python scripts/assets/update_asset_paths.py --target localhost --dry-run
"""

import argparse
import os
import re
import sys
from pathlib import Path

# Configuration
INTERNAL_SERVER = "omniverse://10.50.2.21"
LOCALHOST_SERVER = "omniverse://localhost"

# Get the repository root (parent of scripts directory)
REPO_ROOT = Path(__file__).parent.parent.parent

# Files to update (relative to repo root)
FILES_TO_UPDATE = [
    "costnav_isaacsim/launch.py",
    "costnav_isaacsim/config/config_loader.py",
    "costnav_isaacsim/config/mission_config.yaml",
    "costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v1_CustomMap/costnav_isaaclab_env_cfg.py",
    "costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL/costnav_isaaclab_env_cfg.py",
    "costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL/coco_robot_cfg.py",
    "costnav_isaaclab/source/costnav_isaaclab/costnav_isaaclab/tasks/manager_based/costnav_isaaclab_v2_NavRL/check_navmesh.py",
]


def get_local_path(omni_path: str) -> str:
    """Convert omniverse:// path to local file path."""
    # Extract the path after the server
    relative = omni_path.replace(f"{INTERNAL_SERVER}/", "")
    # Return path relative to assets directory
    return str(REPO_ROOT / "assets" / relative)


def update_file(file_path: Path, old_pattern: str, new_pattern: str, dry_run: bool) -> bool:
    """Update a single file, replacing old_pattern with new_pattern."""
    if not file_path.exists():
        print(f"  SKIP: File not found: {file_path}")
        return False

    content = file_path.read_text()
    
    if old_pattern not in content:
        print(f"  SKIP: Pattern not found in {file_path.name}")
        return False

    new_content = content.replace(old_pattern, new_pattern)
    
    if dry_run:
        print(f"  WOULD UPDATE: {file_path}")
        # Show diff
        for line_num, (old_line, new_line) in enumerate(
            zip(content.splitlines(), new_content.splitlines()), 1
        ):
            if old_line != new_line:
                print(f"    Line {line_num}:")
                print(f"      - {old_line.strip()}")
                print(f"      + {new_line.strip()}")
    else:
        file_path.write_text(new_content)
        print(f"  UPDATED: {file_path}")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Update Omniverse asset paths in codebase")
    parser.add_argument(
        "--target",
        choices=["localhost", "local", "internal"],
        required=True,
        help="Target path type: localhost (omniverse://localhost), local (file paths), internal (original server)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show changes without applying them",
    )
    args = parser.parse_args()

    print("=" * 60)
    print("CostNav Asset Path Updater")
    print("=" * 60)
    print(f"Target: {args.target}")
    print(f"Dry run: {args.dry_run}")
    print("=" * 60)

    if args.target == "localhost":
        old_pattern = INTERNAL_SERVER
        new_pattern = LOCALHOST_SERVER
    elif args.target == "internal":
        old_pattern = LOCALHOST_SERVER
        new_pattern = INTERNAL_SERVER
    else:  # local file paths - more complex, needs per-file handling
        print("ERROR: Local file path mode not yet implemented.")
        print("Use --target localhost with a local Nucleus server instead.")
        sys.exit(1)

    updated_count = 0
    for rel_path in FILES_TO_UPDATE:
        file_path = REPO_ROOT / rel_path
        print(f"\nProcessing: {rel_path}")
        if update_file(file_path, old_pattern, new_pattern, args.dry_run):
            updated_count += 1

    print("\n" + "=" * 60)
    action = "Would update" if args.dry_run else "Updated"
    print(f"{action} {updated_count} files")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())

