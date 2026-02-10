"""Evaluation state and manager."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Optional

from .metrics import init_property_contact_counts
from . import food, metrics


@dataclass
class EvaluationState:
    """State for evaluation metrics and tracking."""

    impulse_min_threshold: float = 50.0
    impulse_health_max: float = 177.8
    impulse_health: float = 177.8
    impulse_damage_accumulated: float = 0.0
    contact_report_subscription: Optional[object] = None
    contact_report_targets: set[str] = field(default_factory=set)

    contact_count: int = 0
    total_impulse: float = 0.0
    last_contact_count: Optional[int] = None
    last_total_impulse: Optional[float] = None
    people_contact_count: int = 0
    last_people_contact_count: Optional[int] = None
    property_contact_counts: dict[str, int] = field(default_factory=init_property_contact_counts)
    last_property_contact_counts: Optional[dict[str, int]] = None
    property_contact_impulse_min_threshold: float = 100.0

    last_damage_steps_remaining: int = 0
    damage_cooldown_steps: int = 30

    delta_v_magnitudes_mps: list[float] = field(default_factory=list)
    last_delta_v_magnitudes_mps: Optional[list[float]] = None
    injury_costs: list[float] = field(default_factory=list)
    total_injury_cost: float = 0.0
    last_injury_costs: Optional[list[float]] = None
    last_total_injury_cost: Optional[float] = None

    food_root_prim_path: Optional[str] = None
    food_prefix_path: Optional[str] = None
    food_pieces_prim_path: Optional[str] = None
    food_bucket_prim_path: Optional[str] = None
    initial_food_piece_count: int = 0
    final_food_piece_count: Optional[int] = None


class EvaluationManager:
    """Facade for evaluation tracking and helpers."""

    def __init__(
        self,
        config,
        is_mission_active: Callable[[], bool],
    ) -> None:
        self._config = config
        self._is_mission_active = is_mission_active
        self.state = EvaluationState()

    def reset_impulse_health(self) -> None:
        metrics.reset_impulse_health(self.state)

    def setup_contact_reporting(self) -> None:
        food.update_food_root_prefix(self.state, self._config)
        metrics.setup_contact_reporting(self.state, self._config, self.on_contact_report)

    def on_contact_report(self, contact_headers, contact_data) -> None:
        metrics.on_contact_report(
            self.state,
            contact_headers,
            contact_data,
            self._config,
            self._is_mission_active,
        )

    def handle_simulation_restart(self, stage_reloaded: bool = False) -> None:
        self.reset_impulse_health()
        self.state.contact_report_subscription = None
        self.state.contact_report_targets = set()
        self.setup_contact_reporting()
        if stage_reloaded:
            food.setup_food_tracking(self.state, self._config)

    def count_food_pieces_in_bucket(self) -> int:
        return food.count_food_pieces_in_bucket(self.state, self._config)

    def setup_food_tracking(self) -> None:
        food.setup_food_tracking(self.state, self._config)

    def check_food_spoilage(self) -> bool:
        return food.check_food_spoilage(self.state, self._config)

    def reset_food_for_teleport(self) -> bool:
        return food.reset_food_for_teleport(self.state, self._config)
