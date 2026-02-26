#!/usr/bin/env python3
"""Summarize evaluation log files into CSV and print overall summary.

Usage:
    python scripts/summarize_logs.py \\
        --logs logs/vint_evaluation_20260225_165217.log logs/vint_evaluation_20260225_191924.log \\
        --num-missions 100 \\
        --output logs/combined_summary.csv
"""

import argparse
import csv
import os
import re
import sys


def parse_log(filepath):
    """Parse per-mission result blocks from a single evaluation log file."""
    missions = []
    with open(filepath, "r") as f:
        lines = f.read().split("\n")

    i = 0
    while i < len(lines):
        line = lines[i]
        # Match timestamped "  Mission N:" header (the result block, not progress lines)
        m = re.match(r"\[.*?\]\s+Mission (\d+):\s*$", line)
        if not m:
            i += 1
            continue

        mission_num = int(m.group(1))
        mission = {"mission": mission_num}
        i += 1

        while i < len(lines):
            raw = lines[i]
            # Strip timestamp prefix
            clean = re.sub(r"^\[.*?\]\s*", "", raw).strip()

            # Stop at next mission header or blank line
            if not clean:
                break
            if re.match(r"\[.*?\]\s+Mission \d+", raw) and "Status:" not in clean:
                break

            if clean.startswith("Status:"):
                mission["status"] = clean.split(":", 1)[1].strip()
            elif clean.startswith("Reason:"):
                mission["reason"] = clean.split(":", 1)[1].strip()
            elif clean.startswith("Traveled distance:"):
                mission["distance"] = float(re.search(r"([\d.]+)m", clean).group(1))
            elif clean.startswith("Elapsed time:"):
                mission["time"] = float(re.search(r"([\d.]+)s", clean).group(1))
            elif clean.startswith("Average velocity:"):
                mission["velocity"] = float(re.search(r"[\d.]+", clean.split(":", 1)[1]).group())
            elif clean.startswith("Average mechanical power:"):
                mission["mech_power"] = float(re.search(r"[\d.]+", clean.split(":", 1)[1]).group())
            elif clean.startswith("Contact count:"):
                mission["contact_count"] = int(clean.split(":", 1)[1].strip())
            elif clean.startswith("People contact count:"):
                mission["people_contact"] = int(clean.split(":", 1)[1].strip())
            elif "Property contacts" in clean:

                def _g(pat: str) -> int:
                    m_ = re.search(pat, clean)
                    return int(m_.group(1)) if m_ else 0

                mission["property_contact"] = _g(r"total=(\d+)")
                mission["bollard"] = _g(r"bollard=(\d+)")
                mission["building"] = _g(r"building=(\d+)")
                mission["trash_bin"] = _g(r"trash_bin=(\d+)")
                mission["mail_box"] = _g(r"mail_box=(\d+)")
            elif clean.startswith("Total impulse:"):
                mission["total_impulse"] = float(re.search(r"[\d.]+", clean.split(":", 1)[1]).group())
            elif clean.startswith("Delta-v count:"):
                mission["deltav_count"] = int(re.search(r"Delta-v count:\s*(\d+)", clean).group(1))
            elif clean.startswith("Total injury cost:"):
                mission["injury_cost"] = float(clean.split(":", 1)[1].strip())
            elif clean.startswith("Food spoiled:"):
                mission["food_spoiled"] = clean.split(":", 1)[1].strip().lower() == "true"

            i += 1

        if "status" in mission:
            missions.append(mission)

    return missions


def avg(missions, key, default=0):
    vals = [m.get(key, default) for m in missions]
    return sum(vals) / len(vals) if vals else 0


def fmt(val, decimals=2):
    """Format number: use leading dot for values < 1, otherwise normal."""
    if abs(val) < 1:
        return f".{int(round(val * (10**decimals))):0{decimals}d}"
    return f"{val:.{decimals}f}"


def main():
    parser = argparse.ArgumentParser(description="Summarize evaluation log files into CSV + summary.")
    parser.add_argument("--logs", nargs="+", required=True, help="List of log file paths (in order).")
    parser.add_argument(
        "--num-missions",
        type=int,
        default=None,
        help="Use only the first N missions from the combined list. Default: use all.",
    )
    parser.add_argument(
        "--output", type=str, default=None, help="Output CSV path. Default: logs/evaluation_summary.csv"
    )
    args = parser.parse_args()

    # Parse all logs and concatenate
    all_missions = []
    source_labels = []
    for log_path in args.logs:
        if not os.path.isfile(log_path):
            print(f"ERROR: File not found: {log_path}", file=sys.stderr)
            sys.exit(1)
        parsed = parse_log(log_path)
        label = os.path.basename(log_path)
        for m in parsed:
            m["source_log"] = label
        all_missions.extend(parsed)
        source_labels.append(f"{label} ({len(parsed)} missions)")

    if args.num_missions is not None:
        all_missions = all_missions[: args.num_missions]

    N = len(all_missions)
    if N == 0:
        print("ERROR: No missions parsed from the provided log files.", file=sys.stderr)
        sys.exit(1)

    # Renumber globally
    for idx, m in enumerate(all_missions):
        m["global_mission"] = idx + 1

    # Write CSV
    output_path = args.output or "logs/evaluation_summary.csv"
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fields = [
        "global_mission",
        "source_log",
        "status",
        "reason",
        "distance",
        "time",
        "velocity",
        "mech_power",
        "contact_count",
        "people_contact",
        "property_contact",
        "bollard",
        "building",
        "trash_bin",
        "mail_box",
        "total_impulse",
        "deltav_count",
        "injury_cost",
        "food_spoiled",
    ]
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for m in all_missions:
            writer.writerow(m)

    # Compute stats
    success = sum(1 for m in all_missions if m["status"] == "SUCCESS_SLA")
    food_spoiled = sum(1 for m in all_missions if m["status"] == "FAILURE_FOODSPOILED")
    timeout = sum(1 for m in all_missions if m["status"] == "FAILURE_TIMEOUT")
    phys_assist = sum(1 for m in all_missions if m["status"] == "FAILURE_PHYSICALASSISTANCE")
    skipped = sum(1 for m in all_missions if m["status"] == "SKIPPED")

    non_skipped = [m for m in all_missions if m["status"] != "SKIPPED"]
    ns = len(non_skipped)
    success_rate = (success / ns * 100) if ns > 0 else 0

    def stats_block(subset, label):
        n = len(subset)
        if n == 0:
            return f"  (no data {label})\n"
        lines = []
        lines.append(f"  Avg Distance: {fmt(avg(subset, 'distance'))}m ({label})")
        lines.append(f"  Avg Time:     {fmt(avg(subset, 'time'))}s ({label})")
        lines.append(f"  Avg Velocity: {fmt(avg(subset, 'velocity'), 4)}m/s ({label})")
        lines.append(f"  Avg Mech Power: {fmt(avg(subset, 'mech_power'), 4)}kW ({label})")
        lines.append(f"  Avg Contact Count: {fmt(avg(subset, 'contact_count'))} ({label})")
        lines.append(f"  Avg People Contact Count: {fmt(avg(subset, 'people_contact'))} ({label})")
        ap = avg(subset, "property_contact")
        ab = avg(subset, "bollard")
        abld = avg(subset, "building")
        at = avg(subset, "trash_bin")
        am = avg(subset, "mail_box")
        lines.append(
            f"  Avg Property Contact Count: {fmt(ap)} (bollard={fmt(ab)}, building={fmt(abld)}, trash_bin={fmt(at)}, mail_box={fmt(am)}) ({label})"
        )
        lines.append(f"  Avg Total Impulse: {fmt(avg(subset, 'total_impulse'))} N*s ({label})")
        lines.append(f"  Avg Delta-v Count: {fmt(avg(subset, 'deltav_count'))} ({label})")
        lines.append(f"  Avg Injury Cost: {fmt(avg(subset, 'injury_cost'))} ({label})")
        return "\n".join(lines) + "\n"

    print()
    print("==============================================")
    print("  Evaluation Complete")
    print("==============================================")
    print(f"  SUCCESS_SLA:              {success} / {N}")
    print(f"  FAILURE_FOODSPOILED:      {food_spoiled} / {N}")
    print(f"  FAILURE_TIMEOUT:          {timeout} / {N}")
    print(f"  FAILURE_PHYSICALASSISTANCE: {phys_assist} / {N}")
    print(f"  Skipped:                  {skipped} / {N}")
    print(f"  Success Rate: {fmt(success_rate)}% (excluding skipped)")
    print(stats_block(non_skipped, "excluding skipped"), end="")
    print(stats_block(all_missions, "including skipped"), end="")
    print(f"  Sources: {', '.join(source_labels)}")
    print(f"  CSV output: {output_path}")
    print("==============================================")


if __name__ == "__main__":
    main()
