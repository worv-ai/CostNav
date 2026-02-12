# Copyright (c) 2025, CostNav Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""NavMesh-based topological map generator for ViNT navigation.

This module generates ViNT-compatible topological maps by:
1. Querying NavMesh shortest path between start and goal positions
2. Interpolating waypoints at fixed intervals along the path
3. Capturing RGB images at each waypoint using a virtual camera in Isaac Sim
4. Saving images as sequentially numbered PNGs (0.png, 1.png, ..., N.png)

The output topomap is directly compatible with ViNT's navigate.py which loads
images by sequential index from a directory.

Reference:
    - ViNT navigate.py: third_party/visualnav-transformer/deployment/src/navigate.py
    - NavMesh API: omni.anim.navigation.core
"""

from __future__ import annotations

import logging
import math
import os
from typing import TYPE_CHECKING, List, Optional

from .navmesh_sampler import NAVMESH_AVAILABLE, NavMeshSampler, SampledPosition

if TYPE_CHECKING:
    from costnav_isaacsim.config import TopoMapConfig

logger = logging.getLogger(__name__)


class TopomapGenerator:
    """Generates ViNT-compatible topological maps from NavMesh shortest paths.

    This class combines NavMesh path queries with Isaac Sim's virtual camera
    to produce a sequence of RGB images along the optimal navigation route.

    Example usage:
        from costnav_isaacsim.config import TopoMapConfig
        from costnav_isaacsim.mission_manager import NavMeshSampler, TopomapGenerator

        sampler = NavMeshSampler(min_distance=5.0, max_distance=50.0)
        config = TopoMapConfig(enabled=True, waypoint_interval=0.5)
        generator = TopomapGenerator(sampler, config, simulation_context)

        start, goal = sampler.sample_start_goal_pair()
        saved_paths = generator.generate_topomap(start, goal)
    """

    def __init__(
        self,
        sampler: NavMeshSampler,
        config: "TopoMapConfig",
        simulation_context,
    ):
        """Initialize the topomap generator.

        Args:
            sampler: NavMeshSampler instance for path queries.
            config: TopoMapConfig with camera/waypoint settings.
            simulation_context: Isaac Sim SimulationContext for rendering.
        """
        self._sampler = sampler
        self._config = config
        self._simulation_context = simulation_context

        # Camera state (lazy-initialized via setup_camera)
        self._camera_prim = None
        self._rgb_annotator = None
        self._render_product = None
        self._stage = None

        logger.info(
            f"TopomapGenerator initialized: interval={config.waypoint_interval}m, "
            f"resolution={config.image_width}x{config.image_height}, "
            f"camera_height={config.camera_height_offset}m"
        )

    def get_shortest_path_waypoints(
        self,
        start: SampledPosition,
        goal: SampledPosition,
    ) -> Optional[List[SampledPosition]]:
        """Query NavMesh for shortest path waypoints between start and goal.

        Args:
            start: Start position on the NavMesh.
            goal: Goal position on the NavMesh.

        Returns:
            List of sparse SampledPosition waypoints (at turns/corners),
            or None if no path exists.
        """
        if not NAVMESH_AVAILABLE:
            logger.error("NavMesh extensions not available.")
            return None

        try:
            import carb

            navmesh = self._sampler._get_navmesh()

            start_point = carb.Float3(start.x, start.y, start.z)
            end_point = carb.Float3(goal.x, goal.y, goal.z)

            path = navmesh.query_shortest_path(
                start_pos=start_point,
                end_pos=end_point,
                agent_radius=self._sampler.agent_radius,
            )

            if path is None:
                logger.warning(f"No path found from ({start.x:.2f}, {start.y:.2f}) " f"to ({goal.x:.2f}, {goal.y:.2f})")
                return None

            points = path.get_points()
            if points is None or len(points) == 0:
                logger.warning("Path returned but has no waypoints.")
                return None

            waypoints = []
            for pt in points:
                waypoints.append(
                    SampledPosition(
                        x=float(pt.x),
                        y=float(pt.y),
                        z=float(pt.z),
                    )
                )

            logger.info(
                f"NavMesh path: {len(waypoints)} sparse waypoints from "
                f"({start.x:.2f}, {start.y:.2f}) to ({goal.x:.2f}, {goal.y:.2f})"
            )
            return waypoints

        except Exception as e:
            logger.error(f"Failed to query shortest path: {e}")
            return None

    @staticmethod
    def interpolate_waypoints(
        sparse_points: List[SampledPosition],
        interval: float = 0.5,
    ) -> List[SampledPosition]:
        """Interpolate sparse waypoints at fixed distance intervals.

        Inserts intermediate points between consecutive sparse waypoints
        so that the resulting list has approximately uniform spacing.
        Heading at each point is computed as the direction toward the next point.

        Args:
            sparse_points: Sparse waypoints from NavMesh path query.
            interval: Distance between interpolated waypoints (meters).

        Returns:
            Dense list of SampledPosition with headings set.
        """
        if len(sparse_points) < 2:
            return list(sparse_points)

        dense_points: List[SampledPosition] = []

        for seg_idx in range(len(sparse_points) - 1):
            p0 = sparse_points[seg_idx]
            p1 = sparse_points[seg_idx + 1]

            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            seg_len = math.sqrt(dx * dx + dy * dy)

            if seg_len < 1e-6:
                continue

            heading = math.atan2(dy, dx)
            num_steps = max(1, int(seg_len / interval))

            for step in range(num_steps):
                t = step / num_steps
                dense_points.append(
                    SampledPosition(
                        x=p0.x + t * dx,
                        y=p0.y + t * dy,
                        z=p0.z + t * dz,
                        heading=heading,
                    )
                )

        # Add the final goal point with heading from the last segment
        last = sparse_points[-1]
        if len(sparse_points) >= 2:
            prev = sparse_points[-2]
            last_heading = math.atan2(last.y - prev.y, last.x - prev.x)
        else:
            last_heading = 0.0
        dense_points.append(
            SampledPosition(
                x=last.x,
                y=last.y,
                z=last.z,
                heading=last_heading,
            )
        )

        logger.info(
            f"Interpolated {len(sparse_points)} sparse → {len(dense_points)} dense " f"waypoints (interval={interval}m)"
        )
        return dense_points

    def setup_camera(self) -> None:
        """Create the virtual camera prim and attach a Replicator RGB annotator.

        This must be called before capture_image_at_position(). It creates
        a Camera prim in the USD stage with properties matching the robot's
        rgb_left camera, and sets up the Omni Replicator render product.
        """
        try:
            import omni.replicator.core as rep
            from isaacsim.core.utils import prims as prim_utils
            from pxr import Gf, UsdGeom

            cfg = self._config
            cam_prim_path = cfg.camera_prim_path
            resolution = (cfg.image_width, cfg.image_height)

            # Get USD stage
            if self._stage is None:
                import omni.usd

                self._stage = omni.usd.get_context().get_stage()

            existing_prim = self._stage.GetPrimAtPath(cam_prim_path)
            if existing_prim.IsValid():
                logger.info(f"[TOPOMAP] Reusing existing camera at {cam_prim_path}")
                self._camera_prim = existing_prim
            else:
                self._camera_prim = prim_utils.create_prim(
                    cam_prim_path,
                    prim_type="Camera",
                    translation=(0.0, 0.0, cfg.camera_height_offset),
                    orientation=(0.5, 0.5, 0.5, 0.5),
                )
                camera_geom = UsdGeom.Camera(self._camera_prim)
                camera_geom.GetFocalLengthAttr().Set(cfg.focal_length)
                camera_geom.GetFocusDistanceAttr().Set(cfg.focus_distance)
                camera_geom.GetHorizontalApertureAttr().Set(cfg.horizontal_aperture)
                camera_geom.GetVerticalApertureAttr().Set(cfg.vertical_aperture)
                camera_geom.GetClippingRangeAttr().Set(Gf.Vec2f(0.076, 100000.0))
                logger.info(f"[TOPOMAP] Created camera at {cam_prim_path}")

            self._render_product = rep.create.render_product(
                cam_prim_path,
                resolution=resolution,
            )
            self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
            self._rgb_annotator.attach(self._render_product)

            logger.info(f"[TOPOMAP] Camera setup complete: {resolution[0]}x{resolution[1]} " f"@ {cam_prim_path}")

        except ImportError as exc:
            logger.error(f"[TOPOMAP] Isaac Sim modules not available: {exc}")
            raise
        except Exception as exc:
            logger.error(f"[TOPOMAP] Failed to setup camera: {exc}")
            raise

    def capture_image_at_position(self, position: SampledPosition):
        """Capture an RGB image at the given position and heading.

        Moves the virtual camera to (x, y, z + camera_height_offset) with
        the specified heading, steps the simulation to flush the render
        pipeline, and returns the captured RGB numpy array.

        Args:
            position: SampledPosition with x, y, z, heading.

        Returns:
            numpy.ndarray of shape (H, W, 3) with dtype uint8, or None on failure.
        """
        if self._camera_prim is None or self._rgb_annotator is None:
            logger.error("[TOPOMAP] Camera not initialized. Call setup_camera() first.")
            return None

        try:
            import numpy as np
            from pxr import Gf, UsdGeom
            from transforms3d.euler import euler2quat

            xform = UsdGeom.Xformable(self._camera_prim)

            # Get or create translate operation
            translate_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
            translate_op = translate_ops[0] if translate_ops else xform.AddTranslateOp()

            # Get or create orient operation
            orient_ops = [op for op in xform.GetOrderedXformOps() if op.GetOpType() == UsdGeom.XformOp.TypeOrient]
            orient_op = orient_ops[0] if orient_ops else xform.AddOrientOp()

            # Set camera position
            camera_pos = Gf.Vec3d(
                position.x,
                position.y,
                position.z + self._config.camera_height_offset,
            )
            translate_op.Set(camera_pos)

            # Convert heading (yaw) to quaternion
            # euler2quat returns (w, x, y, z)
            q = euler2quat(0, 0, position.heading)
            yaw_quat = Gf.Quatd(q[0], Gf.Vec3d(q[1], q[2], q[3]))
            # Base camera orientation (ROS convention + 180deg rotation)
            base_quat = Gf.Quatd(0.5, Gf.Vec3d(0.5, 0.5, 0.5))
            final_quat = yaw_quat * base_quat
            orient_op.Set(final_quat)

            # Step simulation to flush render pipeline
            for _ in range(self._config.render_settle_steps):
                self._simulation_context.step(render=True)

            # Capture image
            rgb_data = self._rgb_annotator.get_data()

            if rgb_data is None:
                logger.warning("[TOPOMAP] No image data from annotator")
                return None

            if isinstance(rgb_data, np.ndarray):
                if rgb_data.ndim == 3 and rgb_data.shape[2] == 4:
                    rgb_data = rgb_data[:, :, :3]  # Drop alpha channel
                return rgb_data.astype(np.uint8)
            else:
                logger.warning(f"[TOPOMAP] Unexpected data type: {type(rgb_data)}")
                return None

        except Exception as exc:
            logger.error(f"[TOPOMAP] Failed to capture image: {exc}")
            return None

    def cleanup_camera(self) -> None:
        """Remove the topomap camera prim and release render resources."""
        try:
            if self._render_product is not None:
                import omni.replicator.core as rep

                rep.orchestrator.stop()
                self._render_product = None

            if self._camera_prim is not None and self._stage is not None:
                cam_path = self._config.camera_prim_path
                self._stage.RemovePrim(cam_path)
                logger.info(f"[TOPOMAP] Removed camera prim at {cam_path}")

            self._camera_prim = None
            self._rgb_annotator = None
        except Exception as exc:
            logger.warning(f"[TOPOMAP] Cleanup warning: {exc}")

    def generate_topomap(
        self,
        start: SampledPosition,
        goal: SampledPosition,
        output_dir: Optional[str] = None,
    ) -> List[str]:
        """Generate a complete topological map between start and goal.

        This is the main entry point. It:
        1. Queries NavMesh shortest path
        2. Interpolates waypoints at configured interval
        3. Sets up the virtual camera
        4. Captures an image at each waypoint
        5. Saves images as 0.png, 1.png, ..., N.png
        6. Cleans up the camera

        Args:
            start: Start position on the NavMesh.
            goal: Goal position on the NavMesh.
            output_dir: Directory to save images. If None, uses config default.

        Returns:
            List of saved image file paths, or empty list on failure.
        """
        from PIL import Image

        out_dir = output_dir or self._config.output_dir
        os.makedirs(out_dir, exist_ok=True)

        # Step 1: Query shortest path
        sparse_waypoints = self.get_shortest_path_waypoints(start, goal)
        if sparse_waypoints is None or len(sparse_waypoints) < 2:
            logger.error("[TOPOMAP] Failed to get valid path. Aborting topomap generation.")
            return []

        # Step 2: Interpolate waypoints
        dense_waypoints = self.interpolate_waypoints(
            sparse_waypoints,
            self._config.waypoint_interval,
        )
        if len(dense_waypoints) == 0:
            logger.error("[TOPOMAP] Interpolation produced no waypoints.")
            return []

        logger.info(f"[TOPOMAP] Generating topomap: {len(dense_waypoints)} waypoints → {out_dir}")

        # Step 3: Setup camera
        self.setup_camera()

        # Step 4 & 5: Capture and save images
        saved_paths: List[str] = []
        try:
            for i, waypoint in enumerate(dense_waypoints):
                rgb_data = self.capture_image_at_position(waypoint)
                if rgb_data is None:
                    logger.warning(f"[TOPOMAP] Skipping waypoint {i}: capture failed")
                    continue

                img_path = os.path.join(out_dir, f"{i}.png")
                img = Image.fromarray(rgb_data)
                img.save(img_path)
                saved_paths.append(img_path)

                if (i + 1) % 50 == 0 or i == len(dense_waypoints) - 1:
                    logger.info(f"[TOPOMAP] Progress: {i + 1}/{len(dense_waypoints)} images saved")
        finally:
            # Step 6: Cleanup
            self.cleanup_camera()

        logger.info(f"[TOPOMAP] Topomap generation complete: {len(saved_paths)} images " f"saved to {out_dir}")
        return saved_paths
