#!/usr/bin/env python3
"""
Bootstrap script to download Omniverse assets.

This script properly initializes Isaac Sim before importing omni.client.

Usage (inside Docker):
    docker compose --profile isaac-lab run --rm isaac-lab \
        python /workspace/assets/download_assets.py
"""

import os
import sys

# Must import isaacsim first to bootstrap the environment
print("Initializing Isaac Sim environment...")
try:
    import isaacsim
except ImportError:
    print("ERROR: isaacsim module not found.")
    print("Make sure you're running inside the Isaac Sim/Lab Docker container.")
    sys.exit(1)

from isaacsim import SimulationApp

# Create headless simulation app (required to initialize omni modules)
simulation_app = SimulationApp({"headless": True})

# Now we can import omni.client
import omni.client

from pathlib import Path

# List of all Omniverse assets used in the codebase
OMNIVERSE_ASSETS = [
    "omniverse://10.50.2.21/Users/worv/coco_one_fix_prim.usd",
    "omniverse://10.50.2.21/Users/worv/costnav/foods/popcorn/popcorn.usd",
    "omniverse://10.50.2.21/Users/worv/costnav/street_sidewalk_segwaye1_Corrected.usd",
    "omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd",
    "omniverse://10.50.2.21/Users/worv/map/Street_road.usd",
    "omniverse://10.50.2.21/Users/worv/map/Street_sidewalk.usd",
    "omniverse://10.50.2.21/Users/worv/map/temp.usd",
]

OMNIVERSE_SERVER = "omniverse://10.50.2.21"
OUTPUT_DIR = Path("/workspace/assets")


def download_asset(url: str, output_dir: Path) -> bool:
    """Download a single asset from Omniverse to local filesystem."""
    relative_path = url.replace(f"{OMNIVERSE_SERVER}/", "")
    local_path = output_dir / relative_path
    
    local_path.parent.mkdir(parents=True, exist_ok=True)
    
    print(f"Downloading: {url}")
    print(f"        To: {local_path}")
    
    result, _, content = omni.client.read_file(url)
    
    if result != omni.client.Result.OK:
        print(f"  ERROR: Failed to read {url} - {result}")
        return False
    
    with open(local_path, "wb") as f:
        f.write(memoryview(content))
    
    print(f"  SUCCESS: Downloaded {len(content)} bytes")
    return True


def download_with_dependencies(url: str, output_dir: Path, downloaded: set) -> bool:
    """Download an asset and related files in the same directory."""
    if url in downloaded:
        return True
    
    downloaded.add(url)
    
    if not download_asset(url, output_dir):
        return False
    
    # For USD files, download sibling files (textures, materials, etc.)
    if url.endswith((".usd", ".usda", ".usdc")):
        parent_url = "/".join(url.split("/")[:-1])
        result, entries = omni.client.list(parent_url)
        
        if result == omni.client.Result.OK:
            for entry in entries:
                entry_url = f"{parent_url}/{entry.relative_path}"
                if entry_url not in downloaded:
                    # Check if it's a file (not a directory)
                    if not (entry.flags & omni.client.ItemFlags.CAN_HAVE_CHILDREN):
                        download_asset(entry_url, output_dir)
                        downloaded.add(entry_url)
    
    return True


def main():
    print("=" * 60)
    print("Omniverse Asset Downloader")
    print("=" * 60)
    print(f"Server: {OMNIVERSE_SERVER}")
    print(f"Output: {OUTPUT_DIR.absolute()}")
    print(f"Assets: {len(OMNIVERSE_ASSETS)}")
    print("=" * 60)
    
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    
    downloaded = set()
    success_count = 0
    fail_count = 0
    
    for url in OMNIVERSE_ASSETS:
        print("")
        if download_with_dependencies(url, OUTPUT_DIR, downloaded):
            success_count += 1
        else:
            fail_count += 1
    
    print("")
    print("=" * 60)
    print(f"Download complete: {success_count} succeeded, {fail_count} failed")
    print(f"Total files downloaded: {len(downloaded)}")
    print("=" * 60)
    
    # Cleanup
    simulation_app.close()
    
    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

