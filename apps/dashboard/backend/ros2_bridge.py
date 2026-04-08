"""ROS2 bridge — thin wrapper around rclpy for the dashboard backend.

Runs rclpy.spin in a background thread so FastAPI's asyncio loop is not blocked.
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from std_srvs.srv import Trigger

logger = logging.getLogger("dashboard.ros2_bridge")

ROLLING_RESISTANCE_FORCE = 18.179  # Newtons (for mech power calc)


class ROS2Bridge:
    """Singleton ROS2 node that exposes async helpers for FastAPI."""

    _instance: ROS2Bridge | None = None

    def __init__(self) -> None:
        rclpy.init()
        self._node = Node("dashboard_bridge")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # Publishers
        self._mission_index_pub = self._node.create_publisher(Int32, "/set_mission_index", 10)
        self._timeout_pub = self._node.create_publisher(Float64, "/set_mission_timeout", 10)

        # Service clients
        self._start_cli = self._node.create_client(Trigger, "/start_mission")
        self._skip_cli = self._node.create_client(Trigger, "/skip_mission")
        self._result_cli = self._node.create_client(Trigger, "/get_mission_result")

        # Spin in background thread
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        logger.info("ROS2 bridge initialized")

    @classmethod
    def get(cls) -> ROS2Bridge:
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            logger.exception("ROS2 spin error")

    # --- async helpers (called from FastAPI) --------------------------------

    async def set_mission_index(self, idx: int) -> None:
        msg = Int32()
        msg.data = idx
        await asyncio.to_thread(self._mission_index_pub.publish, msg)

    async def set_timeout(self, timeout: float) -> None:
        msg = Float64()
        msg.data = timeout
        await asyncio.to_thread(self._timeout_pub.publish, msg)

    async def start_mission(self) -> dict:
        return await self._call_trigger(self._start_cli)

    async def skip_mission(self) -> dict:
        return await self._call_trigger(self._skip_cli)

    async def get_mission_result(self) -> dict:
        resp = await self._call_trigger(self._result_cli)
        # The mission manager encodes the result as JSON in the message field
        if resp.get("success") and resp.get("message"):
            try:
                return json.loads(resp["message"])
            except (json.JSONDecodeError, TypeError):
                pass
        return resp

    async def _call_trigger(self, client) -> dict:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=2.0):
                return {"success": False, "message": "Service not available"}

        def _call():
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
            if future.done():
                resp = future.result()
                return {"success": resp.success, "message": resp.message}
            return {"success": False, "message": "Service call timed out"}

        return await asyncio.to_thread(_call)

    def shutdown(self) -> None:
        self._executor.shutdown()
        self._node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def parse_result(raw: dict, mission_id: int, difficulty: str | None = None) -> dict:
        """Convert raw /get_mission_result JSON into a structured dict."""
        elapsed = float(raw.get("elapsed_time", 0) or 0)
        traveled = float(raw.get("traveled_distance", 0) or 0)
        avg_vel = traveled / elapsed if elapsed > 0 else 0.0
        avg_mech = ROLLING_RESISTANCE_FORCE * avg_vel / 1000 if elapsed > 0 else 0.0

        return {
            "mission_id": mission_id,
            "mission_number": mission_id + 1,
            "difficulty": difficulty,
            "result": raw.get("result", "pending"),
            "result_reason": raw.get("result_reason"),
            "in_progress": raw.get("in_progress", False),
            "distance_to_goal": float(raw.get("distance_to_goal", 0) or 0),
            "traveled_distance": traveled,
            "elapsed_time": elapsed,
            "avg_velocity": round(avg_vel, 4),
            "avg_mech_power": round(avg_mech, 4),
            "total_contact_count": int(raw.get("total_contact_count", 0) or 0),
            "total_impulse": float(raw.get("total_impulse", 0) or 0),
            "people_contact_count": int(raw.get("people_contact_count", 0) or 0),
            "property_contacts": {
                "total": int(raw.get("property_contact_total", 0) or 0),
                "fire_hydrant": int(raw.get("property_contact_fire_hydrant", 0) or 0),
                "traffic_light": int(raw.get("property_contact_traffic_light", 0) or 0),
                "street_lamp": int(raw.get("property_contact_street_lamp", 0) or 0),
                "bollard": int(raw.get("property_contact_bollard", 0) or 0),
                "building": int(raw.get("property_contact_building", 0) or 0),
                "trash_bin": int(raw.get("property_contact_trash_bin", 0) or 0),
                "mail_box": int(raw.get("property_contact_mail_box", 0) or 0),
                "newspaper_box": int(raw.get("property_contact_newspaper_box", 0) or 0),
                "bus_stop": int(raw.get("property_contact_bus_stop", 0) or 0),
            },
            "delta_v_count": int(raw.get("delta_v_count", 0) or 0),
            "delta_v_avg_mps": float(raw.get("delta_v_avg_mps", 0) or 0),
            "delta_v_avg_mph": float(raw.get("delta_v_avg_mph", 0) or 0),
            "injury_cost": float(raw.get("total_injury_cost", 0) or 0),
            "food": {
                "enabled": raw.get("food_enabled", False),
                "initial_pieces": int(raw.get("food_initial_pieces", -1) or -1),
                "final_pieces": int(raw.get("food_final_pieces", -1) or -1),
                "loss_fraction": float(raw.get("food_loss_fraction", -1) or -1),
                "spoiled": raw.get("food_spoiled", False),
            },
        }
