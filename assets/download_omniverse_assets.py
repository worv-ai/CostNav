#!/usr/bin/env python3
"""
Download Omniverse assets from the internal server to local tmp folder.

This script downloads all assets referenced in the codebase from
omniverse://10.50.2.21/ to ./assets/ maintaining the same directory structure.

Usage:
    # Inside Isaac Sim environment:
    python assets/download_omniverse_assets.py

    # Or with Isaac Sim Python:
    /isaac-sim/python.sh assets/download_omniverse_assets.py

    # Or using isaacsim module directly:
    python -c "import isaacsim; exec(open('assets/download_omniverse_assets.py').read())"
"""

import os
import sys
from pathlib import Path

# Try to bootstrap Isaac Sim environment if not already done
def setup_isaac_sim():
    """Setup Isaac Sim environment for omni.client access."""
    isaac_sim_path = os.environ.get("ISAAC_PATH", "/isaac-sim")

    # Add Isaac Sim paths if not already present
    paths_to_add = [
        os.path.join(isaac_sim_path, "kit", "kernel", "py"),
        os.path.join(isaac_sim_path, "kit", "plugins"),
        os.path.join(isaac_sim_path, "kit", "exts"),
        os.path.join(isaac_sim_path, "kit", "extscore"),
        os.path.join(isaac_sim_path, "exts"),
        os.path.join(isaac_sim_path, "extscache"),
    ]

    for p in paths_to_add:
        if os.path.exists(p) and p not in sys.path:
            sys.path.insert(0, p)


setup_isaac_sim()

# List of all Omniverse assets used in the codebase
OMNIVERSE_ASSETS = [
    "omniverse://10.50.2.21/Users/worv/costnav/foods/popcorn/popcorn.usd",
    "omniverse://10.50.2.21/Users/worv/costnav/street_sidewalk_segwaye1_Corrected.usd",
    "omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd",
]

OMNIVERSE_SERVER = "omniverse://10.50.2.21"
OUTPUT_DIR = Path("./assets")


def download_asset(omni_client, url: str, output_dir: Path) -> bool:
    """Download a single asset from Omniverse to local filesystem."""
    # Extract relative path from URL
    relative_path = url.replace(f"{OMNIVERSE_SERVER}/", "")
    local_path = output_dir / relative_path
    
    # Create parent directories
    local_path.parent.mkdir(parents=True, exist_ok=True)
    
    print(f"Downloading: {url}")
    print(f"        To: {local_path}")
    
    # Read from Omniverse
    result, _, content = omni_client.read_file(url)
    
    if result != omni_client.Result.OK:
        print(f"  ERROR: Failed to read {url} - {result}")
        return False
    
    # Write to local file
    with open(local_path, "wb") as f:
        f.write(memoryview(content))
    
    print(f"  SUCCESS: Downloaded {len(content)} bytes")
    return True


def download_asset_with_dependencies(omni_client, url: str, output_dir: Path, downloaded: set, include_siblings: bool = True) -> bool:
    """Download an asset and optionally its sibling files (textures, materials, etc.)."""
    if url in downloaded:
        return True

    downloaded.add(url)

    if not download_asset(omni_client, url, output_dir):
        return False

    # Optionally download sibling files for USD assets
    if include_siblings and url.endswith((".usd", ".usda", ".usdc")):
        relative_path = url.replace(f"{OMNIVERSE_SERVER}/", "")
        local_path = output_dir / relative_path

        # List directory to find related assets (textures, materials, etc.)
        parent_url = "/".join(url.split("/")[:-1])
        result, entries = omni_client.list(parent_url)

        if result == omni_client.Result.OK:
            for entry in entries:
                entry_url = f"{parent_url}/{entry.relative_path}"
                if entry_url not in downloaded and not entry.flags & omni_client.ItemFlags.CAN_HAVE_CHILDREN:
                    # It's a file, download it
                    download_asset(omni_client, entry_url, output_dir)
                    downloaded.add(entry_url)

    return True


def get_omni_client():
    """Try multiple methods to import omni.client."""
    # Method 1: Direct import (works if PYTHONPATH is set correctly)
    try:
        import omni.client as omni_client
        return omni_client
    except ImportError:
        pass

    # Method 2: Try importing via isaacsim bootstrap
    try:
        import isaacsim
        from omni.isaac.kit import SimulationApp
        # Create a minimal simulation app to initialize omni
        simulation_app = SimulationApp({"headless": True})
        import omni.client as omni_client
        return omni_client
    except ImportError:
        pass

    # Method 3: Try importing carb first (lower-level approach)
    try:
        import carb
        import omni.client as omni_client
        return omni_client
    except ImportError:
        pass

    return None


def main():
    omni_client = get_omni_client()

    if omni_client is None:
        print("ERROR: omni.client not available.")
        print("This script must be run inside Isaac Sim environment.")
        print("")
        print("Try one of these methods:")
        print("")
        print("  Method 1 - Using Isaac Sim python.sh:")
        print("    /isaac-sim/python.sh assets/download_omniverse_assets.py")
        print("")
        print("  Method 2 - Using isaacsim module:")
        print("    python -m isaacsim assets/download_omniverse_assets.py")
        print("")
        print("  Method 3 - Inside Docker with proper environment:")
        print("    docker compose --profile isaac-lab run --rm isaac-lab bash")
        print("    # Then inside container:")
        print("    /isaac-sim/python.sh /workspace/assets/download_omniverse_assets.py")
        print("")
        print("  Method 4 - Source Isaac Sim environment first:")
        print("    source /isaac-sim/setup_conda_env.sh")
        print("    python assets/download_omniverse_assets.py")
        sys.exit(1)

    # Initialize Omniverse client
    omni_client.initialize()
    
    print("=" * 60)
    print("Omniverse Asset Downloader")
    print("=" * 60)
    print(f"Server: {OMNIVERSE_SERVER}")
    print(f"Output: {OUTPUT_DIR.absolute()}")
    print(f"Assets: {len(OMNIVERSE_ASSETS)}")
    print("=" * 60)
    
    # Create output directory
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    
    downloaded = set()
    success_count = 0
    fail_count = 0
    
    for url in OMNIVERSE_ASSETS:
        print("")
        if download_asset_with_dependencies(omni_client, url, OUTPUT_DIR, downloaded):
            success_count += 1
        else:
            fail_count += 1
    
    print("")
    print("=" * 60)
    print(f"Download complete: {success_count} succeeded, {fail_count} failed")
    print(f"Total files downloaded: {len(downloaded)}")
    print("=" * 60)
    
    # Shutdown client
    omni_client.shutdown()
    
    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

