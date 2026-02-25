#!/usr/bin/env python3
"""
Download pretrained baseline checkpoints from Hugging Face.

This script downloads baseline model checkpoints (ViNT, NoMaD, GNM) from
https://huggingface.co/maum-ai/CostNav_baseline to ./checkpoints/.

Usage:
    python scripts/assets/download_baseline_checkpoints_hf.py
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
REPO_ID = "maum-ai/CostNav_baseline"
REPO_TYPE = "model"
# Get the repository root (parent of scripts directory)
REPO_ROOT = Path(__file__).parent.parent.parent
CHECKPOINTS_DIR = REPO_ROOT / "checkpoints"


def main():
    print("=" * 60)
    print("Hugging Face Baseline Checkpoint Downloader")
    print("=" * 60)
    print(f"Repository: https://huggingface.co/{REPO_ID}")
    print(f"Output Dir: {CHECKPOINTS_DIR.absolute()}")
    print("=" * 60)

    print("\n" + "=" * 60)
    print("Downloading baseline checkpoints with snapshot_download...")
    print("=" * 60)

    try:
        snapshot_download(
            repo_id=REPO_ID,
            repo_type=REPO_TYPE,
            local_dir=CHECKPOINTS_DIR,
        )
    except Exception as e:
        print(f"ERROR: Snapshot download failed: {e}")
        print("\nMake sure the repository exists and is accessible.")
        return 1

    print("\n" + "=" * 60)
    print("Download complete!")
    print(f"Checkpoints saved to: {CHECKPOINTS_DIR}")
    print("")
    print("Expected files:")
    print(f"  {CHECKPOINTS_DIR / 'baseline-vint.pth'}")
    print(f"  {CHECKPOINTS_DIR / 'baseline-gnm.pth'}")
    print(f"  {CHECKPOINTS_DIR / 'baseline-nomad.pth'}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
