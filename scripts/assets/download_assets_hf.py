#!/usr/bin/env python3
"""
Download Omniverse assets from Hugging Face datasets.

This script downloads assets from https://huggingface.co/datasets/maum-ai/CostNav
to ./assets/Users/ maintaining the same directory structure.

Usage:
    python scripts/assets/download_assets_hf.py
"""

import sys
from pathlib import Path

try:
    from huggingface_hub import snapshot_download
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


def main():
    print("=" * 60)
    print("Hugging Face Asset Downloader")
    print("=" * 60)
    print(f"Repository: https://huggingface.co/datasets/{REPO_ID}")
    print(f"Output Dir: {ASSETS_DIR.absolute()}")
    print("=" * 60)

    print("\n" + "=" * 60)
    print("Downloading assets with snapshot_download (entire repo)...")
    print("=" * 60)

    try:
        snapshot_download(
            repo_id=REPO_ID,
            repo_type=REPO_TYPE,
            local_dir=ASSETS_DIR,
        )
    except Exception as e:
        print(f"ERROR: Snapshot download failed: {e}")
        print("\nMake sure the repository exists and is accessible.")
        return 1

    print("\n" + "=" * 60)
    print("Download complete")
    print(f"Assets saved to: {ASSETS_DIR}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
