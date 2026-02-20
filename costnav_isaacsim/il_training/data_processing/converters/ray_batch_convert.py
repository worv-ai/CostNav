#!/usr/bin/env python3
"""Ray-based batch conversion of ROS bags to MediaRef format.

This script enables parallel processing of multiple ROS bags using Ray
for distributed execution across CPU cores.

Example:
    # Process all bags in a directory
    python ray_batch_convert.py \
        --input_dir data/sample_rosbags \
        --output_dir data/processed \
        --config configs/processing_config.yaml \
        --num_workers 8

    # Process specific bags
    python ray_batch_convert.py \
        --input_dir data/sample_rosbags \
        --output_dir data/processed \
        --bags recording_001 recording_002
"""

from __future__ import annotations

import argparse
import csv
import logging
import sys
import time
import warnings
from pathlib import Path
from typing import Optional

import ray
from rich.console import Console
from rich.table import Table

try:
    # When run as a module
    from il_training.data_processing.converters.rosbag_to_mediaref import (
        convert_bag,
        detect_format,
        load_config,
    )
except ImportError:
    # When run as a script from the converters directory
    from rosbag_to_mediaref import convert_bag, detect_format, load_config

console = Console()


@ray.remote
def convert_bag_task(
    input_path: Path,
    output_dir: Path,
    fps: float,
    image_topics: Optional[list[str]],
    topics_to_remove: Optional[list[str]],
) -> dict:
    """Ray task for converting a single bag.

    Args:
        input_path: Path to input bag directory
        output_dir: Output directory for this bag (e.g., base_dir/bag_name_mediaref/)
        fps: Video frame rate
        image_topics: List of image topics to extract
        topics_to_remove: List of topics to exclude

    Returns:
        Dict with conversion status and statistics
    """
    try:
        bag_name = input_path.name
        output_path = output_dir / f"{bag_name}.mcap"
        media_dir = output_dir / f"{bag_name}.media"

        # Skip if already processed
        success_marker = output_dir / "_SUCCESS"
        if success_marker.exists():
            return {"input_path": str(input_path), "status": "skipped", "reason": "already processed"}

        output_dir.mkdir(parents=True, exist_ok=True)
        media_dir.mkdir(parents=True, exist_ok=True)

        stats = convert_bag(
            input_path=input_path,
            output_path=output_path,
            media_dir=media_dir,
            fps=fps,
            image_topics=image_topics,
            topics_to_remove=topics_to_remove,
            quiet=True,
        )

        # Write success marker
        success_marker.write_text(f"Conversion completed\nFrames: {stats['frames_written']}\n")
        stats["status"] = "success"
        return stats

    except Exception as e:
        return {"input_path": str(input_path), "status": "error", "error": str(e)}


def load_csv_filter(csv_path: Path) -> list[str]:
    """Read a CSV file and return recording_dir values where error == 'O'.

    Args:
        csv_path: Path to the CSV file with ``recording_dir`` and ``error`` columns.

    Returns:
        List of recording directory relative paths (e.g. ``"changhoon/recording_..."``).
    """
    filtered: list[str] = []
    with open(csv_path, newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            if row.get("error", "").strip() == "O":
                rec_dir = row.get("recording_dir", "").strip()
                if rec_dir:
                    filtered.append(rec_dir)
    console.print(f"[green]CSV filter: {len(filtered)} recordings with error='O' out of total rows[/green]")
    return filtered


def find_bags(input_dir: Path, bag_names: Optional[list[str]] = None) -> list[Path]:
    """Find all valid ROS bags in directory, recursing into subdirectories."""
    bags = []

    if bag_names:
        # Process specific bags
        for name in bag_names:
            bag_path = input_dir / name
            if bag_path.exists():
                try:
                    detect_format(bag_path)
                    bags.append(bag_path)
                except ValueError:
                    console.print(f"[yellow]Warning: {name} is not a valid bag[/yellow]")
    else:
        # Find all bags in directory, recursing into subdirectories
        for item in input_dir.iterdir():
            if item.is_dir():
                try:
                    detect_format(item)
                    bags.append(item)
                except ValueError:
                    # Not a valid bag — recurse into it to find bags inside
                    sub_bags = find_bags(item)
                    bags.extend(sub_bags)

    return sorted(bags)


def main():
    parser = argparse.ArgumentParser(
        description="Batch convert ROS bags to MediaRef format using Ray",
    )
    parser.add_argument("--input_dir", "-i", type=Path, help="Input directory with bags (can be set in config)")
    parser.add_argument("--output_dir", "-o", type=Path, help="Output directory (can be set in config)")
    parser.add_argument("--config", "-c", type=Path, help="Processing config YAML")
    parser.add_argument("--num_workers", "-n", type=int, help="Number of parallel workers (can be set in config)")
    parser.add_argument("--bags", nargs="*", help="Specific bag names to process (default: all, can be set in config)")
    parser.add_argument(
        "--csv-filter",
        type=Path,
        default=None,
        help="Path to CSV file for filtering bags. Only recordings where error=='O' will be processed.",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Video FPS")

    args = parser.parse_args()

    # Load config
    config = load_config(args.config)

    # Get paths from config if not provided via command line
    paths_config = config.get("paths", {})
    input_dir = args.input_dir or (Path(paths_config["input_dir"]) if "input_dir" in paths_config else None)
    output_dir = args.output_dir or (Path(paths_config["output_dir"]) if "output_dir" in paths_config else None)
    bags = args.bags if args.bags is not None else paths_config.get("bags")

    # Apply CSV filter if provided (overrides --bags / config bags)
    csv_filter = args.csv_filter or (
        Path(paths_config["csv_filter"]) if paths_config.get("csv_filter") else None
    )
    if csv_filter is not None:
        csv_filter = Path(csv_filter)
        if not csv_filter.exists():
            console.print(f"[red]Error: CSV filter file not found: {csv_filter}[/red]")
            sys.exit(1)
        if bags:
            console.print("[yellow]Warning: csv_filter overrides bags list[/yellow]")
        bags = load_csv_filter(csv_filter)

    # Validate required paths
    if input_dir is None:
        console.print("[red]Error: input_dir must be specified via --input_dir or in config file[/red]")
        sys.exit(1)
    if output_dir is None:
        console.print("[red]Error: output_dir must be specified via --output_dir or in config file[/red]")
        sys.exit(1)
    if not input_dir.exists():
        console.print(f"[red]Error: Input directory not found: {input_dir}[/red]")
        sys.exit(1)

    # Get processing settings from config
    fps = config.get("video", {}).get("fps", args.fps)
    image_topics = config.get("image_topics")
    topics_to_remove = config.get("topics_to_remove")
    num_workers = (
        args.num_workers or paths_config.get("num_workers") or config.get("ray", {}).get("max_disk_workers", 4)
    )

    # Find bags to process
    bag_paths = find_bags(input_dir, bags)
    if not bag_paths:
        console.print("[yellow]No bags found to process[/yellow]")
        sys.exit(0)

    console.print(f"[bold]Found {len(bag_paths)} bags to process[/bold]")
    output_dir.mkdir(parents=True, exist_ok=True)

    # Initialize Ray
    # Exclude project metadata so that Ray workers don't trigger uv to
    # recreate the virtual-env inside their temp directory (the relative
    # paths in pyproject.toml would not resolve there).
    warnings.filterwarnings("ignore", message=".*RAY_ACCEL_ENV_VAR_OVERRIDE_ON_ZERO.*")
    ray.init(
        num_cpus=num_workers,
        ignore_reinit_error=True,
        logging_level=logging.WARNING,
        runtime_env={
            "excludes": [
                "pyproject.toml",
                "uv.lock",
                ".venv/",
            ],
        },
    )
    console.print(f"[green]Ray initialized with {num_workers} workers[/green]")

    # Submit tasks - each bag gets its own _mediaref subdirectory
    # Flat output: if names collide across subdirs, append _1, _2, etc.
    start_time = time.time()
    futures = []
    used_names: dict[str, int] = {}  # base_name -> count of times seen
    for bag_path in bag_paths:
        base_name = bag_path.name
        count = used_names.get(base_name, 0)
        used_names[base_name] = count + 1
        suffix = f"_{count}" if count > 0 else ""
        bag_output_dir = output_dir / f"{base_name}{suffix}_mediaref"
        future = convert_bag_task.remote(bag_path, bag_output_dir, fps, image_topics, topics_to_remove)
        futures.append((f"{base_name}{suffix}", future))

    # Collect results
    results = []
    total = len(futures)
    pending = {f: name for name, f in futures}
    while pending:
        done, _ = ray.wait(list(pending.keys()), num_returns=1, timeout=1.0)
        for future in done:
            name = pending.pop(future)
            result = ray.get(future)
            results.append(result)
            completed = total - len(pending)
            elapsed = time.time() - start_time
            avg = elapsed / completed
            eta = avg * len(pending)

            def _fmt_duration(secs: float) -> str:
                """Format seconds as human-readable (Xd )HH:MM:SS."""
                s = int(secs)
                days, s = divmod(s, 86400)
                hours, s = divmod(s, 3600)
                minutes, s = divmod(s, 60)
                if days > 0:
                    return f"{days}d {hours:02d}:{minutes:02d}:{s:02d}"
                return f"{hours:02d}:{minutes:02d}:{s:02d}"

            print(
                f"  [{completed}/{total}] Completed: {name}  "
                f"(elapsed {_fmt_duration(elapsed)}, ETA {_fmt_duration(eta)})",
                flush=True,
            )

    elapsed = time.time() - start_time

    # Print summary
    console.print("\n[bold]Conversion Summary[/bold]")
    table = Table()
    table.add_column("Bag")
    table.add_column("Status")
    table.add_column("Frames")
    table.add_column("Original")
    table.add_column("Output")
    table.add_column("Reduction")

    success_count = 0
    error_count = 0
    skipped_count = 0
    total_input_size = 0
    total_output_size = 0

    mb = 1024 * 1024

    for result in results:
        bag_name = Path(result["input_path"]).name
        status = result["status"]

        if status == "success":
            frames = sum(result.get("frames_written", {}).values())
            input_size = result.get("input_size", 0)
            total_size = result.get("total_size", 0)
            reduction = result.get("reduction_percent", 0)
            total_input_size += input_size
            total_output_size += total_size
            table.add_row(
                bag_name,
                "[green]✓ Success[/green]",
                str(frames),
                f"{input_size / mb:.1f} MB",
                f"{total_size / mb:.1f} MB",
                f"[cyan]{reduction:.1f}%[/cyan]",
            )
            success_count += 1
        elif status == "skipped":
            table.add_row(bag_name, "[yellow]⊘ Skipped[/yellow]", "-", "-", "-", "-")
            skipped_count += 1
        else:
            table.add_row(bag_name, f"[red]✗ Error: {result.get('error', 'unknown')}[/red]", "-", "-", "-", "-")
            error_count += 1

    console.print(table)
    console.print(f"\n[bold]Total: {len(results)} bags in {elapsed:.1f}s[/bold]")
    console.print(f"  Success: {success_count}, Skipped: {skipped_count}, Errors: {error_count}")
    if total_input_size > 0:
        total_reduction = (1 - total_output_size / total_input_size) * 100
        console.print(
            f"  Size: {total_input_size / mb:.1f} MB → {total_output_size / mb:.1f} MB "
            f"([cyan]{total_reduction:.1f}% reduction[/cyan])"
        )

    ray.shutdown()


if __name__ == "__main__":
    main()
