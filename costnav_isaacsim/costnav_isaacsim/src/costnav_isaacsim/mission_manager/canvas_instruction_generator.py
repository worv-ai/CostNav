# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Canvas instruction generator for CANVAS integration.

This module converts NavMesh shortest paths into pixel-space trajectory
annotations and publishes them as ROS2 messages for CANVAS:

1. Queries the NavMesh shortest path between start and goal positions.
2. Converts world-coordinate waypoints to pixel coordinates using
   map metadata loaded from a Nav2 map YAML file (resolution, origin).
3. Builds a Scenario JSON string (no CANVAS imports required).
4. Publishes the scenario on ``/instruction_scenario`` (String) and
   the trajectory on ``/instruction_annotation`` (Int32MultiArray).
5. Sends ``/start_pause`` (Bool) to start the planner.
6. Monitors ``/reached_goal`` and ``/model_state`` for completion.

this module only handles the *publishing* side of the interface.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import TYPE_CHECKING, List, Tuple

import numpy as np
import yaml

from .navmesh_sampler import NavMeshSampler, SampledPosition

if TYPE_CHECKING:
    from rclpy.node import Node

    from costnav_isaacsim.config import CanvasInstructionConfig

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Map YAML helpers
# ---------------------------------------------------------------------------


def _load_map_yaml(yaml_path: str) -> dict:
    """Load a Nav2-format map YAML and return parsed metadata.

    Returns a dict with keys: ``resolution``, ``origin`` (list[float]),
    ``image`` (resolved absolute path to image file).

    Raises:
        FileNotFoundError: If *yaml_path* does not exist.
        KeyError: If required fields are missing.
    """
    path = Path(yaml_path)
    if not path.exists():
        raise FileNotFoundError(f"Map YAML not found: {yaml_path}")

    with open(path) as f:
        data = yaml.safe_load(f)

    # Resolve the image path relative to the YAML directory
    image_file = data["image"]
    if not Path(image_file).is_absolute():
        image_file = str(path.parent / image_file)

    return {
        "resolution": float(data["resolution"]),
        "origin": [float(v) for v in data["origin"]],
        "image": image_file,
    }


def _get_image_dimensions(image_path: str) -> Tuple[int, int]:
    """Return (width, height) of the image at *image_path*.

    Uses PIL if available, otherwise falls back to OpenCV.
    """
    try:
        from PIL import Image

        with Image.open(image_path) as img:
            return img.size  # (width, height)
    except ImportError:
        pass

    import cv2

    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {image_path}")
    h, w = img.shape[:2]
    return w, h


# ---------------------------------------------------------------------------
# Coordinate conversion (mirrors SketchMap.pose2coords)
# ---------------------------------------------------------------------------


def pose2pixel(
    x: float,
    y: float,
    resolution: float,
    origin_x: float,
    origin_y: float,
    map_height: int,
) -> Tuple[int, int]:
    """Convert world coordinates to pixel coordinates.


    Args:
        x: World x (meters).
        y: World y (meters).
        resolution: Meters per pixel.
        origin_x: Map origin x (meters).
        origin_y: Map origin y (meters).
        map_height: Image height in pixels.

    Returns:
        (pixel_x, pixel_y) tuple of integers.
    """
    px = int(round((x - origin_x) / resolution))
    py = int(round(map_height - (y - origin_y) / resolution))
    return px, py


class CanvasInstructionGenerator:
    """Generates and publishes canvas instructions for CANVAS.

    Example usage::

        from costnav_isaacsim.config import CanvasInstructionConfig
        from costnav_isaacsim.mission_manager import NavMeshSampler

        generator = CanvasInstructionGenerator(sampler, config, ros_node)
        ok = generator.generate_and_publish(start, goal, mission_id=1)

    The generator:

    * queries the NavMesh shortest path,
    * converts the path to pixel coordinates (via map YAML metadata),
    * builds a Scenario JSON (plain ``json.dumps``, no CANVAS imports),
    * publishes ``/instruction_scenario`` and ``/instruction_annotation``,
    * and optionally sends ``/start_pause True`` to start the planner.
    """

    def __init__(
        self,
        sampler: NavMeshSampler,
        config: "CanvasInstructionConfig",
        node: "Node",
    ) -> None:
        """Initialise the generator.

        Args:
            sampler: NavMeshSampler for shortest-path queries.
            config: CanvasInstructionConfig with topic names & map path.
            node: ROS2 Node to create publishers / subscribers on.
        """
        from std_msgs.msg import Bool, Int32MultiArray, String

        self._sampler = sampler
        self._config = config
        self._node = node

        # --- Load map metadata from YAML -----------------------------------
        map_meta = _load_map_yaml(config.map_yaml_path)
        self._resolution: float = map_meta["resolution"]
        self._origin_x: float = map_meta["origin"][0]
        self._origin_y: float = map_meta["origin"][1]
        w, h = _get_image_dimensions(map_meta["image"])
        self._map_width: int = w
        self._map_height: int = h

        logger.info(
            f"[CANVAS] Map loaded from {config.map_yaml_path}: "
            f"resolution={self._resolution}, origin=({self._origin_x}, {self._origin_y}), "
            f"size={self._map_width}x{self._map_height}"
        )

        # --- Publishers -----------------------------------------------------
        self._scenario_pub = node.create_publisher(String, config.scenario_topic, 10)
        self._annotation_pub = node.create_publisher(Int32MultiArray, config.annotation_topic, 10)
        self._start_pause_pub = node.create_publisher(Bool, config.start_pause_topic, 10)
        self._stop_model_pub = node.create_publisher(Bool, config.stop_model_topic, 10)

        # --- Subscribers (state monitoring) ---------------------------------
        self._reached_goal: bool = False
        self._model_state: str = "init"

        node.create_subscription(Bool, config.reached_goal_topic, self._on_reached_goal, 10)
        node.create_subscription(String, config.model_state_topic, self._on_model_state, 10)

        # --- Mission counter for semantic_uuid ------------------------------
        self._mission_counter: int = 0

    # ------------------------------------------------------------------
    # ROS2 callbacks
    # ------------------------------------------------------------------

    def _on_reached_goal(self, msg) -> None:
        self._reached_goal = msg.data

    def _on_model_state(self, msg) -> None:
        self._model_state = msg.data

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def reached_goal(self) -> bool:
        """Whether model has signalled goal reached."""
        return self._reached_goal

    @property
    def model_state(self) -> str:
        """Latest model_state string from model."""
        return self._model_state

    def reset_state(self) -> None:
        """Reset per-mission tracking state."""
        self._reached_goal = False
        self._model_state = "init"

    def generate_and_publish(
        self,
        start: SampledPosition,
        goal: SampledPosition,
        mission_id: int = 0,
    ) -> bool:
        """Generate canvas instructions and publish to model.

        Returns True on success, False if no valid path was found.
        """
        # 1. Get NavMesh shortest path
        path = self._sampler.get_shortest_path(start, goal)
        if path is None or len(path) < 2:
            logger.warning("[CANVAS] No valid NavMesh path between start and goal")
            return False

        # 2. Convert path to pixel coordinates
        pixel_coords = self._convert_path_to_pixels(path)
        logger.info(f"[CANVAS] Path has {len(path)} waypoints → {pixel_coords.shape[0]} pixels")

        # 3. Build Scenario JSON
        scenario_json = self._build_scenario_json(mission_id)

        # 4. Publish scenario and annotation
        self._publish_scenario_and_annotation(scenario_json, pixel_coords)

        self._mission_counter += 1
        return True

    def send_start(self) -> None:
        """Send start_pause=True to start the model."""
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = True
        self._start_pause_pub.publish(msg)
        logger.info("[CANVAS] Sent start_pause=True")

    def send_stop(self) -> None:
        """Send stop_model=True to stop the model."""
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = True
        self._stop_model_pub.publish(msg)
        logger.info("[CANVAS] Sent stop_model=True")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _convert_path_to_pixels(self, path: List[SampledPosition]) -> np.ndarray:
        """Convert world-coordinate waypoints to pixel coordinates.

        Returns an (N, 2) int32 ndarray of [pixel_x, pixel_y] pairs.
        """
        pixels = [
            pose2pixel(
                pt.x,
                pt.y,
                self._resolution,
                self._origin_x,
                self._origin_y,
                self._map_height,
            )
            for pt in path
        ]
        return np.array(pixels, dtype=np.int32)

    def _build_scenario_json(self, mission_id: int) -> str:
        """Build a Scenario-compatible JSON string (no CANVAS import)."""
        cfg = self._config
        scenario = {
            "semantic_uuid": f"costnav_mission_{mission_id}",
            "map_name": cfg.map_name,
            "sketch_map_name": cfg.sketch_map_name,
            "drive_map_name": cfg.drive_map_name,
            "model_guideline": cfg.model_guideline,
            "map_metadata": {},
        }
        return json.dumps(scenario)

    def _publish_scenario_and_annotation(
        self,
        scenario_json: str,
        pixel_coords: np.ndarray,
    ) -> None:
        """Publish the scenario string and the trajectory annotation."""
        from std_msgs.msg import Int32MultiArray, MultiArrayDimension, MultiArrayLayout, String

        # Scenario
        scenario_msg = String()
        scenario_msg.data = scenario_json
        self._scenario_pub.publish(scenario_msg)
        logger.info(f"[CANVAS] Published scenario on {self._config.scenario_topic}")

        # Annotation: flatten (N, 2) → [x0, y0, x1, y1, ...]
        flat = pixel_coords.flatten().tolist()
        annotation_msg = Int32MultiArray()
        annotation_msg.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label="points", size=pixel_coords.shape[0], stride=pixel_coords.shape[0] * 2),
                MultiArrayDimension(label="xy", size=2, stride=2),
            ],
            data_offset=0,
        )
        annotation_msg.data = flat
        self._annotation_pub.publish(annotation_msg)
        logger.info(
            f"[CANVAS] Published annotation ({pixel_coords.shape[0]} waypoints) " f"on {self._config.annotation_topic}"
        )
