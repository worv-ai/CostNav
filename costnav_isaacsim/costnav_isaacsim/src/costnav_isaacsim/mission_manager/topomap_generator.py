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

import glob
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
        config = TopoMapConfig(enabled=True, waypoint_interval=2.0)
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
            List of raw SampledPosition waypoints from the NavMesh path
            query (typically very dense), or None if no path exists.
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
                f"NavMesh path: {len(waypoints)} raw waypoints from "
                f"({start.x:.2f}, {start.y:.2f}) to ({goal.x:.2f}, {goal.y:.2f})"
            )
            return waypoints

        except Exception as e:
            logger.error(f"Failed to query shortest path: {e}")
            return None

    @staticmethod
    def resample_waypoints(
        raw_points: List[SampledPosition],
        interval: float = 2.0,
    ) -> List[SampledPosition]:
        """Resample waypoints along the polyline at fixed arc-length intervals.

        The NavMesh path query typically returns a very dense set of points.
        This method walks along the polyline and emits a new waypoint every
        *interval* meters of arc length, producing a **uniformly-spaced**
        subset that is usually much smaller than the input.

        Heading at each emitted point is the direction of the segment it
        falls on.

        Args:
            raw_points: Raw waypoints from NavMesh path query.
            interval: Target distance between resampled waypoints (meters).

        Returns:
            List of uniformly-spaced SampledPosition with headings set.
        """
        if len(raw_points) < 2:
            return list(raw_points)

        # --- Build cumulative arc-length table --------------------------------
        seg_lengths: List[float] = []
        for i in range(len(raw_points) - 1):
            p0 = raw_points[i]
            p1 = raw_points[i + 1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            seg_lengths.append(math.sqrt(dx * dx + dy * dy))

        total_length = sum(seg_lengths)
        if total_length < 1e-6:
            return [raw_points[0], raw_points[-1]]

        # --- Walk along the polyline, emitting a point every *interval* m -----
        resampled: List[SampledPosition] = []

        # Always emit the start point
        first = raw_points[0]
        dx0 = raw_points[1].x - first.x
        dy0 = raw_points[1].y - first.y
        resampled.append(
            SampledPosition(
                x=first.x,
                y=first.y,
                z=first.z,
                heading=math.atan2(dy0, dx0),
            )
        )

        next_emit_dist = interval  # arc-length at which the next point fires
        cumulative = 0.0  # arc-length consumed so far

        for seg_idx, seg_len in enumerate(seg_lengths):
            if seg_len < 1e-9:
                cumulative += seg_len
                continue

            p0 = raw_points[seg_idx]
            p1 = raw_points[seg_idx + 1]
            dx = p1.x - p0.x
            dy = p1.y - p0.y
            dz = p1.z - p0.z
            heading = math.atan2(dy, dx)

            seg_start_cum = cumulative
            seg_end_cum = cumulative + seg_len

            # Emit all threshold crossings that fall inside this segment
            while next_emit_dist <= seg_end_cum:
                t = (next_emit_dist - seg_start_cum) / seg_len
                resampled.append(
                    SampledPosition(
                        x=p0.x + t * dx,
                        y=p0.y + t * dy,
                        z=p0.z + t * dz,
                        heading=heading,
                    )
                )
                next_emit_dist += interval

            cumulative = seg_end_cum

        # Always include the final goal point
        last = raw_points[-1]
        prev = raw_points[-2]
        last_heading = math.atan2(last.y - prev.y, last.x - prev.x)
        # Avoid duplicate if the last emitted point is very close to the goal
        if resampled:
            lp = resampled[-1]
            dist_to_last = math.sqrt((last.x - lp.x) ** 2 + (last.y - lp.y) ** 2)
            if dist_to_last > interval * 0.1:
                resampled.append(SampledPosition(x=last.x, y=last.y, z=last.z, heading=last_heading))
        else:
            resampled.append(SampledPosition(x=last.x, y=last.y, z=last.z, heading=last_heading))

        logger.info(f"Resampled {len(raw_points)} raw → {len(resampled)} uniform " f"waypoints (interval={interval}m)")
        return resampled

    def setup_camera(self) -> None:
        """Create the virtual camera prim and attach a Replicator RGB annotator.

        This must be called before capture_image_at_position(). It creates
        a Camera prim in the USD stage with properties matching the robot's
        rgb_left camera, and sets up the Omni Replicator render product.

        The camera prim and render product are kept alive across missions
        to avoid orphaned render products that fall back to the perspective
        camera.  Only the annotator is re-created per mission.
        """
        try:
            import omni.replicator.core as rep

            cfg = self._config
            cam_prim_path = cfg.camera_prim_path
            resolution = (cfg.image_width, cfg.image_height)

            # Get USD stage
            if self._stage is None:
                import omni.usd

                self._stage = omni.usd.get_context().get_stage()

            # --- Camera prim (reuse across missions) ---
            if self._camera_prim is not None and self._camera_prim.IsValid():
                logger.info(f"[TOPOMAP] Reusing camera prim at {self._camera_prim.GetPath()}")
            else:
                existing_prim = self._stage.GetPrimAtPath(cam_prim_path)
                if existing_prim.IsValid():
                    logger.info(f"[TOPOMAP] Reusing existing camera at {cam_prim_path}")
                    self._camera_prim = existing_prim
                elif cfg.camera_usd_path:
                    # Load camera from USD reference (duplicates the camera asset)
                    self._camera_prim = self._create_camera_from_usd(cam_prim_path, cfg.camera_usd_path)
                else:
                    raise RuntimeError(
                        "[TOPOMAP] camera_usd_path is required. "
                        "Set it in mission_config.yaml or via DEFAULT_CAMERA_USD_PATHS in robot_config.py."
                    )

            # Use the actual camera prim path (may differ from cam_prim_path
            # when camera_usd_path is used and the Camera is a descendant)
            actual_cam_path = str(self._camera_prim.GetPath())

            # --- Render product (reuse across missions) ---
            if self._render_product is None:
                self._render_product = rep.create.render_product(
                    actual_cam_path,
                    resolution=resolution,
                )
                logger.info(f"[TOPOMAP] Created render product for {actual_cam_path}")

            # --- Annotator (re-created each mission) ---
            if self._rgb_annotator is None:
                self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb", device="cpu")
                self._rgb_annotator.attach(self._render_product)

            logger.info(f"[TOPOMAP] Camera setup complete: {resolution[0]}x{resolution[1]} " f"@ {actual_cam_path}")

        except ImportError as exc:
            logger.error(f"[TOPOMAP] Isaac Sim modules not available: {exc}")
            raise
        except Exception as exc:
            logger.error(f"[TOPOMAP] Failed to setup camera: {exc}")
            raise

    def _create_camera_from_usd(self, cam_prim_path: str, camera_usd_path: str):
        """Create a camera prim by referencing an external camera USD asset.

        Loads the camera USD file and finds the Camera prim within it.
        The resulting prim inherits all camera intrinsics (focal_length,
        aperture, etc.) from the referenced USD, ensuring exact match
        with the robot's actual camera.

        Args:
            cam_prim_path: USD stage path for the new camera prim.
            camera_usd_path: Asset path to the camera USD file
                (e.g. ``omniverse://localhost/.../camera.usd``).

        Returns:
            The Camera prim (may be a child of the referenced root).
        """
        from pxr import Usd, UsdGeom

        # Create a container prim and add the camera USD as a reference
        prim = self._stage.DefinePrim(cam_prim_path)
        prim.GetReferences().AddReference(camera_usd_path)
        logger.info(f"[TOPOMAP] Added camera USD reference: {camera_usd_path} → {cam_prim_path}")

        # Find the actual Camera prim (may be the prim itself or a descendant)
        if prim.IsA(UsdGeom.Camera):
            logger.info(f"[TOPOMAP] Camera prim loaded at {cam_prim_path}")
            return prim

        for descendant in Usd.PrimRange(prim):
            if descendant.IsA(UsdGeom.Camera):
                logger.info(f"[TOPOMAP] Found Camera prim at {descendant.GetPath()}")
                return descendant

        # No Camera found — fall back to using the root prim and log a warning
        logger.warning(f"[TOPOMAP] No Camera prim found in {camera_usd_path}; " f"using root prim at {cam_prim_path}")
        return prim

    def capture_image_at_position(self, position: SampledPosition, extra_settle_steps: int = 0):
        """Capture an RGB image at the given position and heading.

        Moves the virtual camera to (x, y, z + camera_height_offset) with
        the specified heading, steps the simulation to flush the render
        pipeline, and returns the captured RGB numpy array.

        Args:
            position: SampledPosition with x, y, z, heading.
            extra_settle_steps: Additional simulation steps to run before
                capture (on top of ``render_settle_steps``).  Useful for the
                first image where the render pipeline may not yet be warm.

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
            # Add π to heading so the camera faces forward (toward the next
            # waypoint) instead of backward.
            q = euler2quat(0, 0, position.heading + math.pi)

            # USD xformOp:orient type varies (GfQuatf or GfQuatd) depending
            # on how the prim was created.  Detect the actual attribute type
            # and construct the matching quaternion so Set() never fails.
            attr_type = orient_op.GetAttr().GetTypeName()
            if attr_type == "quatd":
                yaw_quat = Gf.Quatd(float(q[0]), Gf.Vec3d(float(q[1]), float(q[2]), float(q[3])))
                base_quat = Gf.Quatd(0.5, Gf.Vec3d(0.5, 0.5, 0.5))
            else:
                yaw_quat = Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]), float(q[3])))
                base_quat = Gf.Quatf(0.5, Gf.Vec3f(0.5, 0.5, 0.5))
            final_quat = yaw_quat * base_quat
            orient_op.Set(final_quat)

            # Step simulation to flush render pipeline
            total_steps = self._config.render_settle_steps + extra_settle_steps
            for _ in range(total_steps):
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
        """Release per-mission render resources while keeping infrastructure alive.

        The camera prim and render product are intentionally kept alive so
        that subsequent missions can reuse them without creating orphaned
        render products that fall back to the perspective camera.

        Only the RGB annotator is detached here to free captured-image
        memory.  It will be re-created in the next ``setup_camera()`` call.
        """
        try:
            if self._rgb_annotator is not None:
                try:
                    self._rgb_annotator.detach(self._render_product)
                except Exception:
                    pass  # best-effort detach
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
        1. Queries NavMesh shortest path (returns dense raw points)
        2. Resamples waypoints at configured uniform interval
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

        # Clean previous topomap images so stale files from earlier missions
        # don't persist and confuse the ViNT node when it reloads.
        if os.path.isdir(out_dir):
            for old_img in glob.glob(os.path.join(out_dir, "*.png")):
                os.remove(old_img)
            logger.info(f"[TOPOMAP] Cleaned previous images from {out_dir}")

        os.makedirs(out_dir, exist_ok=True)

        # Step 1: Query shortest path (returns dense raw points from NavMesh)
        raw_waypoints = self.get_shortest_path_waypoints(start, goal)
        if raw_waypoints is None or len(raw_waypoints) < 2:
            logger.error("[TOPOMAP] Failed to get valid path. Aborting topomap generation.")
            return []

        # Step 2: Resample at uniform intervals
        logger.info(
            f"[TOPOMAP] Using waypoint_interval={self._config.waypoint_interval}m, "
            f"robot_prim_path={getattr(self._config, 'robot_prim_path', None)}"
        )
        waypoints = self.resample_waypoints(
            raw_waypoints,
            self._config.waypoint_interval,
        )
        if len(waypoints) == 0:
            logger.error("[TOPOMAP] Resampling produced no waypoints.")
            return []

        logger.info(f"[TOPOMAP] Generating topomap: {len(waypoints)} waypoints → {out_dir}")

        # Step 3: Setup camera
        self.setup_camera()

        # Step 4 & 5: Capture and save images
        # Skip the first waypoint (index 0) so the topomap starts at a safe
        # distance from the robot, preventing the robot from appearing in the
        # first captured image.  Saved images are still numbered from 0.png.
        waypoints_to_capture = waypoints[1:]
        saved_paths: List[str] = []
        try:
            for i, waypoint in enumerate(waypoints_to_capture):
                extra = self._config.first_image_extra_settle_steps if i == 0 else 0
                rgb_data = self.capture_image_at_position(waypoint, extra_settle_steps=extra)
                if rgb_data is None:
                    logger.warning(f"[TOPOMAP] Skipping waypoint {i}: capture failed")
                    continue

                img_path = os.path.join(out_dir, f"{i}.png")
                img = Image.fromarray(rgb_data)
                img.save(img_path)
                saved_paths.append(img_path)

                if (i + 1) % 50 == 0 or i == len(waypoints_to_capture) - 1:
                    logger.info(f"[TOPOMAP] Progress: {i + 1}/{len(waypoints_to_capture)} images saved")
        finally:
            # Step 6: Cleanup
            self.cleanup_camera()

        logger.info(f"[TOPOMAP] Topomap generation complete: {len(saved_paths)} images " f"saved to {out_dir}")
        return saved_paths
