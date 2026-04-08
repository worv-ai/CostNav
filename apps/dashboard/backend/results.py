"""Structured result writer — saves evaluation results as JSON and CSV."""

from __future__ import annotations

import csv
import json
import logging
from datetime import datetime, timezone
from pathlib import Path

logger = logging.getLogger("dashboard.results")


def save_results(
    results: list[dict],
    mode: str,
    timeout: float,
    missions_file: str,
    log_dir: str = "logs",
) -> tuple[str, str]:
    """Write evaluation results to JSON and CSV files.

    Returns (json_path, csv_path).
    """
    log_path = Path(log_dir)
    log_path.mkdir(parents=True, exist_ok=True)
    ts = datetime.now(tz=timezone.utc).strftime("%Y%m%d_%H%M%S")
    base = f"dashboard_{mode}_{ts}"

    # --- JSON ---
    completed = [r for r in results if r["result"] != "pending"]
    successes = [r for r in completed if r["result"] == "success"]
    total = len(results)
    payload = {
        "metadata": {
            "mode": mode,
            "timeout": timeout,
            "missions_file": missions_file,
            "started_at": datetime.now(tz=timezone.utc).isoformat(),
            "total_missions": total,
        },
        "missions": results,
        "summary": {
            "total": total,
            "completed": len(completed),
            "success": len(successes),
            "success_rate": len(successes) / len(completed) if completed else 0,
        },
    }
    json_path = log_path / f"{base}.json"
    json_path.write_text(json.dumps(payload, indent=2))
    logger.info("Saved JSON results to %s", json_path)

    # --- CSV (matches evaluation_summary.csv columns) ---
    csv_path = log_path / f"{base}.csv"
    fieldnames = [
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
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            pc = r.get("property_contacts", {})
            food = r.get("food", {})
            writer.writerow(
                {
                    "global_mission": r["mission_number"],
                    "source_log": f"{base}.json",
                    "status": r["result"].upper().replace("SUCCESS", "SUCCESS_SLA").replace("FAILURE_", "FAILURE_"),
                    "reason": r.get("result_reason", ""),
                    "distance": r.get("traveled_distance", 0),
                    "time": r.get("elapsed_time", 0),
                    "velocity": r.get("avg_velocity", 0),
                    "mech_power": r.get("avg_mech_power", 0),
                    "contact_count": r.get("total_contact_count", 0),
                    "people_contact": r.get("people_contact_count", 0),
                    "property_contact": pc.get("total", 0),
                    "bollard": pc.get("bollard", 0),
                    "building": pc.get("building", 0),
                    "trash_bin": pc.get("trash_bin", 0),
                    "mail_box": pc.get("mail_box", 0),
                    "total_impulse": r.get("total_impulse", 0),
                    "deltav_count": r.get("delta_v_count", 0),
                    "injury_cost": r.get("injury_cost", 0),
                    "food_spoiled": food.get("spoiled", False),
                }
            )
    logger.info("Saved CSV results to %s", csv_path)
    return str(json_path), str(csv_path)
