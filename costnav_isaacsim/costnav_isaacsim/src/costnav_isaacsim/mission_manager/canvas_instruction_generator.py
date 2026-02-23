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
import os
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
        (pixel_y, pixel_x) tuple of integers.
    """
    px = int(round((x - origin_x) / resolution))
    py = int(round(map_height - (y - origin_y) / resolution))
    return py, px


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

        Returns an (N, 2) int32 ndarray of [pixel_y, pixel_x] pairs.
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
        from std_msgs.msg import Int32MultiArray, String

        # Scenario
        self._scenario_pub.publish(String(data=scenario_json))
        logger.info(f"[CANVAS] Published scenario on {self._config.scenario_topic}")

        # Annotation: flatten (N, 2) → [y0, x0, y1, x1, ...]
        self._annotation_pub.publish(Int32MultiArray(data=pixel_coords.flatten().tolist()))
        logger.info(
            f"[CANVAS] Published annotation ({pixel_coords.shape[0]} waypoints) " f"on {self._config.annotation_topic}"
        )

        # Debug: save published data to disk and visualize trajectory
        if self._config.debug_enabled or os.environ.get("CANVAS_DEBUG", "").lower() in ("1", "true"):
            self._debug_save_outputs(scenario_json, pixel_coords)

    # ------------------------------------------------------------------
    # Debug helpers
    # ------------------------------------------------------------------

    def _debug_save_outputs(
        self,
        scenario_json: str,
        pixel_coords: np.ndarray,
    ) -> None:
        """Save published data to disk and render a trajectory visualization.

        Outputs (written to ``self._config.debug_output_dir``):

        * ``debug_scenario.json`` – the Scenario JSON string.
        * ``debug_annotation.npy`` – the (N, 2) pixel-coordinate array.
        * ``debug_trajectory.png`` – trajectory drawn on the base map image.

        The method is a no-op (with a warning) if the output directory cannot
        be created or the map image is missing.
        """
        out_dir = Path(self._config.debug_output_dir)
        try:
            out_dir.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            logger.warning(f"[CANVAS-DEBUG] Cannot create output dir {out_dir}: {exc}")
            return

        # 1. Save scenario JSON
        scenario_path = out_dir / "debug_scenario.json"
        scenario_path.write_text(scenario_json, encoding="utf-8")
        logger.info(f"[CANVAS-DEBUG] Saved scenario JSON → {scenario_path}")

        # 2. Save annotation as NumPy array
        annotation_path = out_dir / "debug_annotation.npy"
        np.save(str(annotation_path), pixel_coords)
        logger.info(f"[CANVAS-DEBUG] Saved annotation ({pixel_coords.shape}) → {annotation_path}")

        # 3. Visualize trajectory on the base map image
        map_image_path = self._config.debug_map_image_path
        if not map_image_path:
            logger.info("[CANVAS-DEBUG] No debug_map_image_path configured; skipping trajectory visualization")
            return

        try:
            import cv2

            base_map = cv2.imread(map_image_path, cv2.IMREAD_COLOR)
            if base_map is None:
                logger.warning(f"[CANVAS-DEBUG] Cannot read map image: {map_image_path}")
                return

            # pixel_coords is (N, 2) with columns [pixel_y, pixel_x];
            # OpenCV expects (x, y) so swap columns for drawing.
            xy_coords = pixel_coords[:, ::-1].copy()  # (N, 2) → [pixel_x, pixel_y]
            pts = xy_coords.reshape(-1, 1, 2)  # shape required by cv2.polylines
            cv2.polylines(base_map, [pts], isClosed=False, color=(0, 0, 255), thickness=2)

            # Draw start (green) and goal (blue) circles
            if len(xy_coords) >= 1:
                start_pt = (int(xy_coords[0][0]), int(xy_coords[0][1]))
                cv2.circle(base_map, start_pt, radius=6, color=(0, 255, 0), thickness=-1)
            if len(xy_coords) >= 2:
                goal_pt = (int(xy_coords[-1][0]), int(xy_coords[-1][1]))
                cv2.circle(base_map, goal_pt, radius=6, color=(255, 0, 0), thickness=-1)

            vis_path = out_dir / "debug_trajectory.png"
            cv2.imwrite(str(vis_path), base_map)
            logger.info(f"[CANVAS-DEBUG] Saved trajectory visualization → {vis_path}")
        except ImportError:
            logger.warning("[CANVAS-DEBUG] cv2 (OpenCV) not available; skipping trajectory visualization")
        except Exception as exc:
            logger.warning(f"[CANVAS-DEBUG] Trajectory visualization failed: {exc}")
