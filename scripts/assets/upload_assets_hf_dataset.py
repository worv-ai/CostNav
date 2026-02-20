#!/usr/bin/env python3
"""
Upload CostNav teleop rosbags to a Hugging Face dataset.

This script reads the rosbags time summary CSV and uploads only folders tagged
with "O" in the `episode_success` column, preserving the directory structure.

Usage:
    # Set HF_TOKEN in .env file, then run via docker or locally:
    python3 scripts/assets/upload_assets_hf_dataset.py
"""

import csv
import math
import os
import sys
from pathlib import Path

try:
    from huggingface_hub import HfApi, login
    try:
        from huggingface_hub import CommitOperationAdd
    except Exception:
        CommitOperationAdd = None
except ImportError:
    print("ERROR: huggingface_hub not installed.")
    print("Install with: pip install huggingface_hub")
    sys.exit(1)

# Configuration
REPO_ID = os.environ.get("HF_REPO_ID", "maum-ai/CostNav-Teleop-Dataset")
REPO_TYPE = "dataset"

DATASET_ROOT = Path(
    os.environ.get(
        "COSTNAV_DATASET_ROOT",
        "/mnt/raid11/workspace/haebin/costnav_recording_v3",
    )
)
SUMMARY_CSV = Path(
    os.environ.get(
        "COSTNAV_SUMMARY_CSV",
        str(
            DATASET_ROOT
            / "costnav data collection_v3 recording - rosbags_time_summary.csv"
        ),
    )
)

TAG_COLUMN = "episode_success"
TAG_VALUE = "O"

UPLOAD_BATCH_SIZE = int(os.environ.get("HF_UPLOAD_BATCH_SIZE", "50"))
LIST_MAX = int(os.environ.get("HF_UPLOAD_LIST_MAX", "30"))


def split_mcap_files(value: str):
    if not value:
        return []
    value = value.strip()
    for sep in (";", ",", "|"):
        if sep in value:
            return [part.strip() for part in value.split(sep) if part.strip()]
    return [value]


def resolve_rel_path(rel_or_abs: str):
    path = Path(rel_or_abs)
    if path.is_absolute():
        try:
            rel = path.relative_to(DATASET_ROOT)
        except ValueError:
            return None, path
        return rel, path
    return path, DATASET_ROOT / path


def collect_files():
    if not SUMMARY_CSV.exists():
        print(f"ERROR: Summary CSV not found: {SUMMARY_CSV}")
        return None

    files_by_repo_path = {}
    missing_files = []
    total_rows = 0
    matched_rows = 0
    seen_dirs = set()

    with SUMMARY_CSV.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            print("ERROR: Summary CSV is empty or missing a header row.")
            return None

        required = {"recording_dir", "metadata_path", "mcap_files", TAG_COLUMN}
        missing_cols = sorted(required - set(reader.fieldnames))
        if missing_cols:
            print("ERROR: Summary CSV missing required columns:")
            print("  " + ", ".join(missing_cols))
            return None

        for row in reader:
            total_rows += 1
            tag = (row.get(TAG_COLUMN) or "").strip()
            if tag != TAG_VALUE:
                continue
            matched_rows += 1

            recording_dir = (row.get("recording_dir") or "").strip()
            metadata_path = (row.get("metadata_path") or "").strip()
            mcap_files = (row.get("mcap_files") or "").strip()

            # Primary behavior: upload the entire recording folder for O-tag rows.
            if recording_dir:
                rel_dir, abs_dir = resolve_rel_path(recording_dir)
                if rel_dir is None:
                    missing_files.append(str(abs_dir))
                else:
                    dir_key = rel_dir.as_posix()
                    if dir_key not in seen_dirs:
                        add_directory(
                            files_by_repo_path, missing_files, rel_dir, abs_dir
                        )
                        seen_dirs.add(dir_key)
                continue

            # Fallback: upload listed files if recording_dir is missing.
            if metadata_path:
                rel, abs_path = resolve_rel_path(metadata_path)
                if rel is None:
                    missing_files.append(str(abs_path))
                else:
                    add_file(files_by_repo_path, missing_files, rel, abs_path)

            if mcap_files:
                for mcap_file in split_mcap_files(mcap_files):
                    rel = Path(mcap_file)
                    abs_path = DATASET_ROOT / rel
                    add_file(files_by_repo_path, missing_files, rel, abs_path)

    files_to_upload = [
        (files_by_repo_path[repo_path], repo_path)
        for repo_path in sorted(files_by_repo_path)
    ]
    return files_to_upload, missing_files, total_rows, matched_rows


def add_file(files_by_repo_path, missing_files, rel_path: Path, abs_path: Path):
    if not abs_path.exists():
        missing_files.append(str(abs_path))
        return
    if not abs_path.is_file():
        missing_files.append(str(abs_path))
        return

    repo_path = rel_path.as_posix()
    existing = files_by_repo_path.get(repo_path)
    if existing and existing != abs_path:
        print("WARNING: Duplicate repo path with different source:")
        print(f"  {repo_path}")
        print(f"    kept: {existing}")
        print(f"    skip: {abs_path}")
        return
    files_by_repo_path[repo_path] = abs_path


def add_directory(files_by_repo_path, missing_files, rel_dir: Path, abs_dir: Path):
    if not abs_dir.exists():
        missing_files.append(str(abs_dir))
        return
    if not abs_dir.is_dir():
        missing_files.append(str(abs_dir))
        return

    for file_path in abs_dir.rglob("*"):
        if not file_path.is_file():
            continue
        rel_path = rel_dir / file_path.relative_to(abs_dir)
        add_file(files_by_repo_path, missing_files, rel_path, file_path)


def chunked(items, size):
    if size <= 0:
        size = 50
    for i in range(0, len(items), size):
        yield items[i : i + size]


def upload_files(api, files_to_upload):
    total = len(files_to_upload)
    if total == 0:
        print("ERROR: No files to upload after filtering.")
        return 1

    if CommitOperationAdd is None:
        print("CommitOperationAdd not available; falling back to upload_file.")
        for idx, (local_path, repo_path) in enumerate(files_to_upload, start=1):
            print(f"Uploading {idx}/{total}: {repo_path}")
            api.upload_file(
                path_or_fileobj=str(local_path),
                path_in_repo=repo_path,
                repo_id=REPO_ID,
                repo_type=REPO_TYPE,
                commit_message=f"Add teleop data ({idx}/{total})",
            )
        return 0

    total_batches = int(math.ceil(total / float(UPLOAD_BATCH_SIZE)))
    for batch_idx, batch in enumerate(chunked(files_to_upload, UPLOAD_BATCH_SIZE), 1):
        print(f"Uploading batch {batch_idx}/{total_batches} ({len(batch)} files)")
        operations = [
            CommitOperationAdd(
                path_in_repo=repo_path,
                path_or_fileobj=str(local_path),
            )
            for local_path, repo_path in batch
        ]
        api.create_commit(
            repo_id=REPO_ID,
            repo_type=REPO_TYPE,
            operations=operations,
            commit_message=f"Add teleop data batch {batch_idx}/{total_batches}",
        )
    return 0


def main():
    print("=" * 60)
    print("Hugging Face Teleop Dataset Uploader")
    print("=" * 60)
    print(f"Repository: https://huggingface.co/datasets/{REPO_ID}")
    print(f"Dataset Root: {DATASET_ROOT}")
    print(f"Summary CSV: {SUMMARY_CSV}")
    print(f"Tag Filter: {TAG_COLUMN} == {TAG_VALUE}")
    print("=" * 60)

    hf_token = os.environ.get("HF_TOKEN")
    if not hf_token:
        print("ERROR: HF_TOKEN environment variable not set.")
        print("Please set HF_TOKEN in your .env file.")
        return 1

    if not DATASET_ROOT.exists():
        print(f"ERROR: Dataset root not found: {DATASET_ROOT}")
        return 1

    files_result = collect_files()
    if not files_result:
        return 1
    files_to_upload, missing_files, total_rows, matched_rows = files_result

    if not files_to_upload:
        print("ERROR: No files found to upload.")
        return 1

    total_size = sum(p.stat().st_size for p, _ in files_to_upload)
    total_size_gb = total_size / (1024 * 1024 * 1024)

    print(f"\nRows in CSV: {total_rows}")
    print(f"Rows tagged '{TAG_VALUE}': {matched_rows}")
    print(f"Files to upload: {len(files_to_upload)}")
    print(f"Total size: {total_size_gb:.2f} GB")

    if missing_files:
        print(f"\nWARNING: {len(missing_files)} files referenced but missing.")
        for missing in missing_files[:LIST_MAX]:
            print(f"  missing: {missing}")
        if len(missing_files) > LIST_MAX:
            print(f"  ... {len(missing_files) - LIST_MAX} more not shown")

    print("\nSample of files to upload:")
    for local_path, repo_path in files_to_upload[:LIST_MAX]:
        size_mb = local_path.stat().st_size / (1024 * 1024)
        print(f"  {repo_path} ({size_mb:.2f} MB)")
    if len(files_to_upload) > LIST_MAX:
        print(f"  ... {len(files_to_upload) - LIST_MAX} more not shown")

    print("\n" + "=" * 60)
    response = input("Proceed with upload? [y/N]: ").strip().lower()
    if response != "y":
        print("Upload cancelled.")
        return 0

    login(token=hf_token)
    api = HfApi(token=hf_token)

    try:
        user_info = api.whoami()
        print(f"\nLogged in as: {user_info['name']}")
    except Exception as e:
        print(f"\nERROR: Authentication failed: {e}")
        return 1

    print(f"\nUploading to {REPO_ID}...")
    try:
        result = upload_files(api, files_to_upload)
    except Exception as e:
        print(f"\nERROR: Upload failed: {e}")
        return 1

    print("\n" + "=" * 60)
    print("Upload complete!")
    print(f"View at: https://huggingface.co/datasets/{REPO_ID}")
    print("=" * 60)
    return result


if __name__ == "__main__":
    sys.exit(main())
