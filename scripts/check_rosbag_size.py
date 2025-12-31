#!/usr/bin/env python3
import argparse
import os
from collections import defaultdict

try:
    from rosbag2_py import (
        ConverterOptions,
        MetadataIo,
        SequentialReader,
        StorageOptions,
        get_default_storage_id,
    )
except ImportError as exc:
    raise SystemExit(
        "rosbag2_py not found. Source your ROS 2 setup file first, e.g.:\n"
        "  source /opt/ros/jazzy/setup.bash\n"
        "Then run this script again."
    ) from exc


def detect_storage_id(bag_path, override):
    if override and override != "auto":
        return override

    try:
        meta = MetadataIo().read_metadata(bag_path)
        if meta.storage_identifier:
            return meta.storage_identifier
    except Exception:
        pass

    if os.path.isfile(bag_path):
        if bag_path.endswith(".mcap"):
            return "mcap"
        if bag_path.endswith(".db3"):
            return "sqlite3"
    elif os.path.isdir(bag_path):
        for name in os.listdir(bag_path):
            if name.endswith(".mcap"):
                return "mcap"
            if name.endswith(".db3"):
                return "sqlite3"

    return get_default_storage_id()


def get_bag_metadata(bag_path):
    try:
        return MetadataIo().read_metadata(bag_path)
    except Exception:
        return None


def get_topic_sizes(bag_path, storage_id):
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )

    size_bytes = defaultdict(int)
    counts = defaultdict(int)

    while reader.has_next():
        topic, data, _t = reader.read_next()
        size_bytes[topic] += len(data)
        counts[topic] += 1

    return size_bytes, counts


def format_mb(size):
    return size / (1024 * 1024)


def build_rows(size_bytes, counts, sort_key):
    total_size = sum(size_bytes.values())
    rows = []
    for topic, size in size_bytes.items():
        percent = (size / total_size) * 100 if total_size > 0 else 0
        rows.append((topic, size, percent, counts.get(topic, 0)))

    if sort_key == "name":
        rows.sort(key=lambda r: r[0])
    elif sort_key == "count":
        rows.sort(key=lambda r: r[3], reverse=True)
    elif sort_key == "percent":
        rows.sort(key=lambda r: r[2], reverse=True)
    else:
        rows.sort(key=lambda r: r[1], reverse=True)

    return rows, total_size


def print_table(rows, total_size, bag_size_bytes):
    if not rows:
        print("No messages found in bag.")
        return

    topic_width = min(max(len(r[0]) for r in rows), 80)
    topic_width = max(topic_width, 10)

    print(f"\n{'TOPIC':<{topic_width}} | {'SIZE (MB)':>10} | {'% TOTAL':>8} | {'COUNT':>8}")
    print("-" * (topic_width + 36))
    for topic, size, percent, count in rows:
        print(f"{topic:<{topic_width}} | {format_mb(size):>10.2f} | {percent:>7.1f}% | {count:>8}")
    print("-" * (topic_width + 36))
    print(f"{'Total serialized size':<{topic_width}} | {format_mb(total_size):>10.2f} | {100.0:>7.1f}% |")
    if bag_size_bytes is not None:
        print(f"{'Bag size (metadata)':<{topic_width}} | {format_mb(bag_size_bytes):>10.2f} |")
    print("Note: sizes are serialized message payload only (no container/index overhead).")


def main():
    parser = argparse.ArgumentParser(description="Compute per-topic serialized size and percentage for ROS 2 bags.")
    parser.add_argument("bag_path", help="Path to ROS 2 bag directory or .mcap/.db3 file")
    parser.add_argument(
        "--storage",
        default="auto",
        choices=["auto", "mcap", "sqlite3"],
        help="Storage plugin (auto detects when possible)",
    )
    parser.add_argument(
        "--sort",
        default="size",
        choices=["size", "name", "count", "percent"],
        help="Sort output by this column",
    )
    args = parser.parse_args()

    storage_id = detect_storage_id(args.bag_path, args.storage)
    print(f"Analyzing {args.bag_path} (storage: {storage_id})...")

    meta = get_bag_metadata(args.bag_path)
    bag_size_bytes = getattr(meta, "bag_size", None) if meta else None

    size_bytes, counts = get_topic_sizes(args.bag_path, storage_id)
    rows, total_size = build_rows(size_bytes, counts, args.sort)
    print_table(rows, total_size, bag_size_bytes)


if __name__ == "__main__":
    main()
