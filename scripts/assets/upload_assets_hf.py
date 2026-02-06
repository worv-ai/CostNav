#!/usr/bin/env python3
"""
Upload Omniverse assets to Hugging Face datasets.

This script uploads all assets from ./assets/ (Game, NVIDIA, Projects, Users)
to https://huggingface.co/datasets/maum-ai/CostNav

Usage:
    # Set HF_TOKEN in .env file, then run via docker:
    make upload-assets-hf
"""

import os
import sys
from pathlib import Path

try:
    from huggingface_hub import HfApi, login
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
    print("Hugging Face Asset Uploader")
    print("=" * 60)
    print(f"Repository: https://huggingface.co/datasets/{REPO_ID}")
    print(f"Assets Dir: {ASSETS_DIR.absolute()}")
    print("=" * 60)

    # Authenticate using HF_TOKEN from environment (loaded from .env)
    hf_token = os.environ.get("HF_TOKEN")
    if not hf_token:
        print("ERROR: HF_TOKEN environment variable not set.")
        print("Please set HF_TOKEN in your .env file.")
        sys.exit(1)

    # Login with token
    login(token=hf_token)

    # Check if assets directory exists
    if not ASSETS_DIR.exists():
        print(f"ERROR: Assets directory not found: {ASSETS_DIR}")
        print("Please run `make download-assets-omniverse` first to download assets.")
        sys.exit(1)

    # Collect all files to upload
    files_to_upload = []
    for file_path in ASSETS_DIR.rglob("*"):
        if file_path.is_file():
            # Path in repo should be relative to assets/ directory
            path_in_repo = str(file_path.relative_to(ASSETS_DIR))
            files_to_upload.append((file_path, path_in_repo))

    if not files_to_upload:
        print("ERROR: No files found to upload.")
        sys.exit(1)

    print(f"\nFound {len(files_to_upload)} files to upload:")
    for local_path, repo_path in files_to_upload:
        size_mb = local_path.stat().st_size / (1024 * 1024)
        print(f"  {repo_path} ({size_mb:.2f} MB)")

    print("\n" + "=" * 60)

    # Confirm upload
    response = input("Proceed with upload? [y/N]: ").strip().lower()
    if response != "y":
        print("Upload cancelled.")
        sys.exit(0)

    # Initialize Hugging Face API with token
    api = HfApi(token=hf_token)

    # Verify authentication
    try:
        user_info = api.whoami()
        print(f"\nLogged in as: {user_info['name']}")
    except Exception as e:
        print(f"\nERROR: Authentication failed: {e}")
        print("Please check your HF_TOKEN in .env file.")
        sys.exit(1)

    # Upload using upload_large_folder to handle large folders and avoid rate limits
    print(f"\nUploading to {REPO_ID}...")
    print("Using large folder upload for better handling...")

    try:
        api.upload_large_folder(
            folder_path=str(ASSETS_DIR),
            repo_id=REPO_ID,
            repo_type=REPO_TYPE,
        )
        print("\nSUCCESS: All files uploaded")
    except Exception as e:
        print(f"\nERROR: Upload failed: {e}")
        return 1

    print("\n" + "=" * 60)
    print("Upload complete!")
    print(f"View at: https://huggingface.co/datasets/{REPO_ID}")
    print("=" * 60)

    return 0


if __name__ == "__main__":
    sys.exit(main())
