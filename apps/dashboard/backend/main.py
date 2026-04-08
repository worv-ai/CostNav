"""CostNav Dashboard API — FastAPI bridge between web UI and ROS2."""

from __future__ import annotations

import asyncio
import json
import logging
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse

from .models import EvalRequest, EvalStatus, MissionResult
from .results import save_results
from .ros2_bridge import ROS2Bridge

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dashboard")

MISSION_FILE = os.environ.get("MISSION_FILE", "config/missions/42.json")
LOG_DIR = os.environ.get("LOG_DIR", "logs")

# --- Global eval state ---
_eval_running = False
_eval_stop_requested = False
_eval_current_id: int | None = None
_eval_completed = 0
_eval_total = 0
_eval_results: list[dict] = []
_eval_current_status: dict | None = None
_eval_task: asyncio.Task | None = None


def _load_missions() -> tuple[list[dict], dict]:
    """Load missions from JSON file. Returns (missions_list, raw_data)."""
    with open(MISSION_FILE) as f:
        data = json.load(f)
    return data["missions"], data


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: init ROS2 bridge
    ROS2Bridge.get()
    yield
    # Shutdown
    ROS2Bridge.get().shutdown()


app = FastAPI(title="CostNav Dashboard", lifespan=lifespan)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# --- Endpoints ---


@app.get("/api/missions")
async def get_missions():
    """Return all missions from the configured JSON file."""
    missions, _ = _load_missions()
    return {"missions": missions, "file": MISSION_FILE}


@app.post("/api/missions/{mission_id}/start")
async def start_single_mission(mission_id: int, timeout: float = 241.0):
    """Set mission index and start a single mission."""
    bridge = ROS2Bridge.get()
    await bridge.set_mission_index(mission_id)
    await asyncio.sleep(0.15)  # let the topic propagate
    await bridge.set_timeout(timeout)
    result = await bridge.start_mission()
    return result


@app.post("/api/eval/start")
async def start_eval(req: EvalRequest):
    """Start a sequential evaluation run."""
    global _eval_running, _eval_stop_requested, _eval_task
    global _eval_completed, _eval_total, _eval_results, _eval_current_id

    if _eval_running:
        return {"error": "Evaluation already running"}

    _eval_running = True
    _eval_stop_requested = False
    _eval_completed = 0
    _eval_total = len(req.mission_ids)
    _eval_results = []
    _eval_current_id = None

    _eval_task = asyncio.create_task(_run_eval(req.mission_ids, req.timeout))
    return {"started": True, "total": _eval_total}


@app.post("/api/eval/stop")
async def stop_eval():
    """Stop the current evaluation run."""
    global _eval_stop_requested
    _eval_stop_requested = True
    return {"stopping": True}


@app.post("/api/skip")
async def skip_mission():
    """Skip the current mission."""
    bridge = ROS2Bridge.get()
    return await bridge.skip_mission()


@app.get("/api/status")
async def get_status():
    """Get current evaluation and mission status."""
    return EvalStatus(
        running=_eval_running,
        current_mission_id=_eval_current_id,
        completed=_eval_completed,
        total=_eval_total,
        results=[MissionResult(**r) for r in _eval_results],
        current_status=MissionResult(**_eval_current_status) if _eval_current_status else None,
    )


@app.get("/api/results")
async def list_results():
    """List available result files."""
    log_path = Path(LOG_DIR)
    files = []
    if log_path.exists():
        for f in sorted(log_path.glob("dashboard_*"), reverse=True):
            files.append({"name": f.name, "size": f.stat().st_size})
    return {"files": files}


@app.get("/api/results/{filename}")
async def download_result(filename: str):
    """Download a specific result file."""
    path = Path(LOG_DIR) / filename
    if not path.exists() or not path.name.startswith("dashboard_"):
        return {"error": "File not found"}
    return FileResponse(path, filename=filename)


@app.websocket("/ws/status")
async def ws_status(websocket: WebSocket):
    """WebSocket endpoint for live status updates."""
    await websocket.accept()
    try:
        while True:
            status = EvalStatus(
                running=_eval_running,
                current_mission_id=_eval_current_id,
                completed=_eval_completed,
                total=_eval_total,
                results=[MissionResult(**r) for r in _eval_results],
                current_status=MissionResult(**_eval_current_status) if _eval_current_status else None,
            )
            await websocket.send_json(status.model_dump())
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        pass


# --- Evaluation loop ---


async def _run_eval(mission_ids: list[int], timeout: float):
    """Run missions sequentially, collecting results."""
    global _eval_running, _eval_completed, _eval_current_id
    global _eval_results, _eval_current_status

    bridge = ROS2Bridge.get()
    missions, _ = _load_missions()
    missions_by_id = {m["id"]: m for m in missions}

    try:
        for idx, mid in enumerate(mission_ids):
            if _eval_stop_requested:
                logger.info("Evaluation stopped by user after %d missions", idx)
                break

            _eval_current_id = mid
            mission = missions_by_id.get(mid, {})
            difficulty = mission.get("difficulty")

            # Set index and timeout, then start
            await bridge.set_mission_index(mid)
            await asyncio.sleep(0.15)
            await bridge.set_timeout(timeout)
            resp = await bridge.start_mission()

            if not resp.get("success"):
                logger.warning("Failed to start mission %d: %s", mid, resp.get("message"))
                _eval_results.append(bridge.parse_result({"result": "failure_timeout"}, mid, difficulty))
                _eval_completed += 1
                continue

            # Poll until mission completes
            safety_timeout = timeout + 30
            elapsed = 0.0
            while elapsed < safety_timeout:
                if _eval_stop_requested:
                    await bridge.skip_mission()
                    break

                await asyncio.sleep(1)
                elapsed += 1

                raw = await bridge.get_mission_result()
                parsed = bridge.parse_result(raw, mid, difficulty)
                _eval_current_status = parsed

                result_status = raw.get("result", "pending")
                if result_status != "pending":
                    break

            # Record result
            final_raw = await bridge.get_mission_result()
            final = bridge.parse_result(final_raw, mid, difficulty)
            _eval_results.append(final)
            _eval_completed += 1
            logger.info(
                "Mission %d: %s (%.1fs, %.1fm)",
                mid + 1,
                final["result"],
                final["elapsed_time"],
                final["traveled_distance"],
            )

            await asyncio.sleep(2)  # brief pause between missions

        # Save results
        if _eval_results:
            save_results(_eval_results, "eval", timeout, MISSION_FILE, LOG_DIR)

    except Exception:
        logger.exception("Evaluation error")
    finally:
        _eval_running = False
        _eval_current_id = None
        _eval_current_status = None
        logger.info("Evaluation finished: %d/%d completed", _eval_completed, _eval_total)
