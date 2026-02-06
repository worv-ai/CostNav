#!/usr/bin/env python3
"""
Download Omniverse assets from Hugging Face datasets.

This script downloads assets from https://huggingface.co/datasets/maum-ai/CostNav
to ./assets/Users/ maintaining the same directory structure.

Usage:
    python scripts/assets/download_assets_hf.py
"""

import os
import sys
from pathlib import Path

try:
    from huggingface_hub import hf_hub_download, list_repo_files
except ImportError:
    print("ERROR: huggingface_hub not installed.")
    print("Install with: pip install huggingface_hub")
    sys.exit(1)

# Configuration
REPO_ID = "maum-ai/CostNav"
REPO_TYPE = "dataset"
# Get the repository root (parent of scripts directory)
REPO_ROOT = Path(__file__).parent.parent.parent
ASSETS_DIR = REPO_ROOT / "assets"
# Only download files under Users/ directory
ASSETS_PREFIX = "Users/"


def main():
    print("=" * 60)
    print("Hugging Face Asset Downloader")
    print("=" * 60)
    print(f"Repository: https://huggingface.co/datasets/{REPO_ID}")
    print(f"Output Dir: {ASSETS_DIR.absolute()}")
    print("=" * 60)
    
    # List all files in the repository
    print("\nFetching file list from Hugging Face...")
    try:
        all_files = list_repo_files(repo_id=REPO_ID, repo_type=REPO_TYPE)
    except Exception as e:
        print(f"ERROR: Failed to list repository files: {e}")
        print("\nMake sure the repository exists and is accessible.")
        sys.exit(1)
    
    # Filter to only asset files (under Users/ directory)
    asset_files = [f for f in all_files if f.startswith(ASSETS_PREFIX)]
    
    if not asset_files:
        print(f"ERROR: No asset files found in repository under '{ASSETS_PREFIX}'")
        sys.exit(1)
    
    print(f"\nFound {len(asset_files)} asset files to download:")
    for f in asset_files:
        print(f"  {f}")
    
    print("\n" + "=" * 60)
    print("Downloading assets...")
    print("=" * 60)
    
    success_count = 0
    fail_count = 0
    
    for file_path in asset_files:
        local_path = ASSETS_DIR / file_path
        local_path.parent.mkdir(parents=True, exist_ok=True)
        
        print(f"\nDownloading: {file_path}")
        try:
            # Download directly to assets/ directory (not HF cache)
            cached_path = hf_hub_download(
                repo_id=REPO_ID,
                filename=file_path,
                repo_type=REPO_TYPE,
                local_dir=ASSETS_DIR,
            )
            print(f"  SUCCESS: {local_path}")
            success_count += 1
        except Exception as e:
            print(f"  ERROR: {e}")
            fail_count += 1
    
    print("\n" + "=" * 60)
    print(f"Download complete: {success_count} succeeded, {fail_count} failed")
    print(f"Assets saved to: {ASSETS_DIR / ASSETS_PREFIX}")
    print("=" * 60)
    
    return 0 if fail_count == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

