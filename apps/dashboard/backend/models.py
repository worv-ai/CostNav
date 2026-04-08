"""Pydantic models for the CostNav dashboard API."""

from __future__ import annotations

from pydantic import BaseModel


class Mission(BaseModel):
    id: int
    difficulty: str
    start: dict
    goal: dict
    waypoints: list[list[float]]


class MissionResult(BaseModel):
    mission_id: int
    mission_number: int
    difficulty: str | None = None
    result: str  # pending, success, failure_timeout, etc.
    result_reason: str | None = None
    in_progress: bool = False
    distance_to_goal: float = 0.0
    traveled_distance: float = 0.0
    elapsed_time: float = 0.0
    avg_velocity: float = 0.0
    avg_mech_power: float = 0.0
    total_contact_count: int = 0
    total_impulse: float = 0.0
    people_contact_count: int = 0
    property_contacts: dict = {}
    delta_v_count: int = 0
    delta_v_avg_mps: float = 0.0
    delta_v_avg_mph: float = 0.0
    injury_cost: float = 0.0
    food: dict = {}


class EvalRequest(BaseModel):
    mission_ids: list[int]  # 0-based mission indices
    timeout: float = 241.0


class EvalStatus(BaseModel):
    running: bool = False
    current_mission_id: int | None = None
    completed: int = 0
    total: int = 0
    results: list[MissionResult] = []
    current_status: MissionResult | None = None
