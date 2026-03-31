from __future__ import annotations

import cv2
import numpy as np
from loguru import logger
from numpy.typing import NDArray
from pydantic import BaseModel

from canvas.agent.neural_planner.map.sketch_map import SketchMap
from canvas.agent.neural_planner.ros_utils import euler_from_quaternion
from canvas.agent.neural_planner.distance import interpolate_trajectory as _interpolate_trajectory


class TrajectoryMapEditorConfig(BaseModel):
    distance_threshold: float = 0.2  # meters

    local_arrow_length: int = 1  # meters
    local_arrow_thickness: float = 3
    # BGRA color
    current_color: tuple[int, int, int, int] = (255, 0, 0, 255)
    line_type: int = cv2.LINE_8
    shift: int = 0
    tip_length: float = 0.5

    # BGRA Color
    human_traj_color: tuple[int, int, int, int] = (0, 0, 255, 255)
    robot_traj_color: tuple[int, int, int, int] = (255, 0, 0, 255)  # Alpha will be adjusted on code
    line_max_alpha: float = 0.7  # Max alpha value for the line
    human_traj_thickness: int = 3
    robot_traj_thickness: int = 5

    map_radius: float = 8  # meters
    output_size: tuple[int, int] = (378, 378)


class TrajectoryMapEditor:
    def __init__(self, canvas: SketchMap, config=TrajectoryMapEditorConfig()) -> None:
        """Load the map image and yml file

        Args:
            map_name: name of the map. Should be one of MAP_LIST.
            path: path to the map folder. If None, it will look for the map in the config folder.
                The folder should contain a map image and a yml file with the name of map.
        """
        self.config = config
        self.canvas = canvas
        self.map_yml = canvas.load_yaml()
        # TODO: deal with this issue more gracefully
        if self.map_yml["resolution"] != 0.05:
            logger.warning(
                f"Map resolution is not 0.05 and is {self.map_yml['resolution']}. Be aware that trajectory thickness, arrow size, etc. are set for 0.05 resolution."
            )
        self.map_image = canvas.load_image()
        # convert PIL Image to cv2
        self.map_image = cv2.cvtColor(np.asarray(self.map_image), cv2.COLOR_RGB2BGRA)

        self.latest_odom = None
        self.robot_trajectory = []
        self.human_trajectory = []

        # Pre-allocate crop buffer to avoid np.full on every call
        radius = int(self.config.map_radius / self.map_yml["resolution"])
        c = self.map_image.shape[2]
        self._crop_template = np.full((radius * 2, radius * 2, c), (127, 127, 127, 255), dtype=self.map_image.dtype)

    def get_local_state(self, detail: bool = False) -> NDArray | None:
        """
        This method return local map array.

        Output numpy array should has 4 channels (RGBA order)
            If the odom is not set, return None
        """
        if self.latest_odom is None:
            logger.warning("Odom is not set. add odom first using append_odom method")
            return None

        # Get the latest odom
        odom = self.latest_odom

        # Translate center point to current odom
        map_array = self._crop_map(self.map_image, odom)

        if detail:
            # Draw local state
            local_state_layer = np.zeros_like(map_array)
            local_state_layer = self._draw_local_state(local_state_layer, odom, is_empty=True)

            # Overlay the local state on the map
            map_array = self.overlay_two_image(map_array, local_state_layer)

            # Convert image BGRA(cv2 format) -> RGBA(PIL.Image format)
            map_array = cv2.cvtColor(map_array, cv2.COLOR_BGRA2RGBA)
            local_state_layer = cv2.cvtColor(local_state_layer, cv2.COLOR_BGRA2RGBA)

            return map_array, local_state_layer, odom
        else:
            # Detailed drawings are executed at _get_local_state
            map_array = self._draw_local_state(map_array, odom, is_empty=False)

            # Convert image BGRA(cv2 format) -> RGBA(PIL.Image format)
            map_array = cv2.cvtColor(map_array, cv2.COLOR_BGRA2RGBA)

            return map_array

    def set_human_trajectory(self, human_traj: np.ndarray):
        """Sets the human trajectory while filtering out closely spaced points.

        Args:
            human_traj (np.ndarray): Array of shape (N, 2) containing the pixel coordinates (y, x) of the human trajectory.
        """
        if human_traj.size == 0:
            self.human_trajectory = []
            return

        distance_threshold = self.config.distance_threshold / self.map_yml["resolution"]
        self.human_trajectory = [human_traj[0]]

        for pixel_point in human_traj[1:-1]:  # Exclude the first and last points initially
            if np.linalg.norm(pixel_point - self.human_trajectory[-1]) >= distance_threshold:
                self.human_trajectory.append(pixel_point)

        # Ensure the last point is always included
        if not np.array_equal(self.human_trajectory[-1], human_traj[-1]):
            self.human_trajectory.append(human_traj[-1])

    def append_odom(self, odom: tuple[NDArray, NDArray]):
        """Append odom to robot trajectory
        Filter out the last odom is close enough to filter threshold

        Args:
            odom: tuples containing position (x,y,z) and orientation (x,y,z,w)
        """
        self.latest_odom = odom
        if len(self.robot_trajectory) > 0:
            last_odom = self.robot_trajectory[-1]
            last_position, _ = last_odom
            position, _ = odom
            distance = np.linalg.norm(position[:2] - last_position[:2])  # ignore z
            if distance < self.config.distance_threshold:
                return
        self.robot_trajectory.append(odom)

    def _convert_quaternion_to_yaw(self, quaternion: NDArray) -> float:
        # Extract the orientation angle in radians
        yaw = euler_from_quaternion(quaternion)[2]
        return yaw

    def _draw_local_state(self, map_array: NDArray, odom: tuple[NDArray, NDArray], is_empty: bool = False) -> NDArray:
        # Draw all elements onto a single overlay canvas, then blend once
        overlay = np.zeros_like(map_array)
        self._draw_trajectory_onto(overlay, self.human_trajectory, odom, "human")
        self._draw_trajectory_onto(overlay, self.robot_trajectory, odom, "robot")
        self._draw_arrow_onto(overlay, odom, self.config.current_color)

        if is_empty:
            return overlay
        return self.overlay_two_image(map_array, overlay)

    def _crop_map(self, image: NDArray, odom: tuple[NDArray, NDArray]) -> np.ndarray:
        """
        Crops the map around the current robot pose. If the crop area extends
        beyond the map boundaries, the outside area is filled with a padding color.
        """
        h, w, c = image.shape
        radius = int(self.config.map_radius / self.map_yml["resolution"])

        # Convert robot pose to pixel coordinates
        position, _ = odom
        x, y, _ = position
        pixel_x, pixel_y = self.canvas.pose2coords((x, y))
        pixel_x, pixel_y = int(pixel_x), int(pixel_y)

        # Copy from pre-allocated template instead of np.full every call
        cropped_map = self._crop_template.copy()

        # Calculate source region bounds in original image
        src_x_start = max(0, pixel_x - radius)
        src_x_end = min(w, pixel_x + radius)
        src_y_start = max(0, pixel_y - radius)
        src_y_end = min(h, pixel_y + radius)

        # Calculate destination region bounds in output canvas
        dst_x_start = max(0, radius - pixel_x)
        dst_x_end = dst_x_start + (src_x_end - src_x_start)
        dst_y_start = max(0, radius - pixel_y)
        dst_y_end = dst_y_start + (src_y_end - src_y_start)

        # Copy valid region from source to destination
        valid_map_region = image[src_y_start:src_y_end, src_x_start:src_x_end]
        if valid_map_region.size > 0:
            cropped_map[dst_y_start:dst_y_end, dst_x_start:dst_x_end] = valid_map_region

        return cropped_map

    def return_map_crop_coords(self) -> np.ndarray:
        """
        Returns the coordinates of the map crop in the original map image.
        """
        h, w, c = self.map_image.shape
        radius = int(self.config.map_radius / self.map_yml["resolution"])

        if self.latest_odom is None:
            logger.warning("Odom is not set. using (0, 0) as the current pose.")
            x, y = 0, 0
        else:
            position, _ = self.latest_odom
            x, y, _ = position
        pixel_x, pixel_y = self.canvas.pose2coords((x, y))
        pixel_x, pixel_y = int(pixel_x), int(pixel_y)

        src_x_start = max(0, pixel_x - radius)
        src_x_end = min(w, pixel_x + radius)
        src_y_start = max(0, pixel_y - radius)
        src_y_end = min(h, pixel_y + radius)

        dst_x_start = max(0, radius - pixel_x)
        dst_x_end = dst_x_start + (src_x_end - src_x_start)
        dst_y_start = max(0, radius - pixel_y)
        dst_y_end = dst_y_start + (src_y_end - src_y_start)

        return np.array(
            [
                src_x_start,
                src_x_end,
                src_y_start,
                src_y_end,
                dst_x_start,
                dst_x_end,
                dst_y_start,
                dst_y_end,
                radius,
                c,
            ],
            dtype=np.int32,
        )

    def calculate_pixel_coords_from_robot_traj(self, robot_trajectory: list[tuple[NDArray, NDArray]], cur_x, cur_y):
        """
        Calculate the pixel coordinates of the robot trajectory, around the current pose
        Args:
            robot_trajectory: The robot trajectory, a list of tuples containing position (x,y,z) and orientation (x,y,z,w)
        Return:
            pixel_coords: The pixel coordinates of the robot trajectory, around the current pose (cur_y, cur_x)
        """
        origin_coords = np.array([[position[0], position[1]] for position, orientation in robot_trajectory])
        # convert to pixel coordinates into [x, y] format
        pixel_coords = self.canvas.pose2coords(origin_coords) - np.array([cur_x, cur_y])
        pixel_coords = pixel_coords.astype(np.int32)
        # add dimension if the shape is (2,)
        if pixel_coords.ndim == 1:
            pixel_coords = pixel_coords[None, :]

        # append 0,0 at the end if the last point is not 0,0
        if not np.all(pixel_coords[-1] == 0):
            pixel_coords_out = np.zeros((pixel_coords.shape[0] + 1, pixel_coords.shape[1]), dtype=np.int32)
            pixel_coords_out[:-1] = pixel_coords
        else:
            pixel_coords_out = pixel_coords
        # change the order to [y, x]
        pixel_coords_out = pixel_coords_out[:, ::-1]
        return pixel_coords_out

    def calculate_in_bound_coordinates(self, radius, pixel_coords):
        """
        Compute line segments (x1, y1, x2, y2) from consecutive pairs of points in
        pixel_coords, skipping any pair where both points lie entirely outside
        the square region [-radius, radius) x [-radius, radius).

        Args:
            pixel_coords : numpy.ndarray
                An Nx2 array of [y, x] pixel coordinates.
            radius : int
                Defines the square region for inclusion.

        Returns:
            numpy.ndarray
                A Kx4 array where each row is [y1, x1, y2, x2] for the line segments
                that remain. In integer pixel coordinates in the range [0, 2*radius).
        """

        n = pixel_coords.shape[0]

        if n < 2:
            # Not enough points to form a line
            return np.empty((0, 4), dtype=pixel_coords.dtype)

        # 1) Identify which points are in the valid region
        #    ( -radius <= y < radius ) and ( -radius <= x < radius )
        in_bounds = (
            (pixel_coords[:, 0] >= -radius)
            & (pixel_coords[:, 0] < radius)
            & (pixel_coords[:, 1] >= -radius)
            & (pixel_coords[:, 1] < radius)
        )

        # 2) Build a mask for the consecutive pairs:
        #    We keep pairs if at least one of the points is in range.
        pair_mask = in_bounds[:-1] | in_bounds[1:]

        # 3) Extract and stack the valid line segments into a new array.
        #    pixel_coords is [y, x], so we'll reorder:
        valid_segments = np.column_stack(
            [
                pixel_coords[:-1, 0][pair_mask],  # y1
                pixel_coords[:-1, 1][pair_mask],  # x1
                pixel_coords[1:, 0][pair_mask],  # y2
                pixel_coords[1:, 1][pair_mask],  # x2
            ]
        )

        valid_segments += radius  # shift to the center of the image
        valid_segments = valid_segments.astype(np.int32)
        return valid_segments

    def line_to_image(self, canvas, line_coords, color, thickness, timewise=True):
        """
        Draw line segments onto the canvas using vectorized numpy rasterization.

        Rasterizes all segments at 1px width using vectorized Bresenham interpolation.
        Thickness is applied externally via cv2.dilate in _draw_trajectory_onto.

        Args:
            canvas: The canvas to draw on (modified in-place)
            line_coords: The line coordinates in (y1, x1, y2, x2) format, shape (N, 4)
            color: The color of the line (BGRA)
            thickness: The thickness of the line
            timewise: If True, the line will be faded out over time
        """
        N = len(line_coords)
        if N == 0:
            return

        color = np.array(color, dtype=np.uint8)
        zero_indices = np.where(color == 0)[0]
        max_alpha = self.config.line_max_alpha
        h, w = canvas.shape[:2]

        # Per-segment fading values REVERSED (new=255, old=0) so that cv2.dilate MAX
        # preserves z-order (newer segments spread over older). Inverted back after dilate.
        fractions = np.arange(1, N + 1, dtype=np.float32) / N
        if timewise:
            seg_fading = (255.0 * fractions).astype(np.uint8)
        else:
            seg_fading = (255.0 * (1.0 - fractions)).astype(np.uint8)
        # Uniform alpha for all segments (line_max_alpha applied to alpha channel only)
        alpha_u8 = np.uint8(255 * max_alpha)

        # Extract segment endpoints
        y1s = line_coords[:, 0]
        x1s = line_coords[:, 1]
        y2s = line_coords[:, 2]
        x2s = line_coords[:, 3]

        # Number of pixels per segment (Bresenham step count)
        steps = np.maximum(np.abs(x2s - x1s), np.abs(y2s - y1s)).astype(np.int32) + 1
        total_pixels = int(steps.sum())

        # Build flat interpolation parameter t for all pixels (vectorized)
        cum_offsets = np.zeros(N, dtype=np.int64)
        np.cumsum(steps[:-1], out=cum_offsets[1:])
        local_idx = np.arange(total_pixels) - np.repeat(cum_offsets, steps)
        denom = np.repeat(np.maximum(steps - 1, 1), steps).astype(np.float32)
        t = local_idx / denom

        # Interpolate all pixel coordinates at once
        x_flat = np.round(np.repeat(x1s, steps) + t * np.repeat(x2s - x1s, steps)).astype(np.int32)
        y_flat = np.round(np.repeat(y1s, steps) + t * np.repeat(y2s - y1s, steps)).astype(np.int32)
        fading_flat = np.repeat(seg_fading, steps)

        # Write 1px thin lines with correct fading and alpha
        valid = (y_flat >= 0) & (y_flat < h) & (x_flat >= 0) & (x_flat < w)
        yv = y_flat[valid]
        xv = x_flat[valid]
        fv = fading_flat[valid]
        canvas[yv, xv] = color
        canvas[yv, xv, zero_indices[0]] = fv
        if len(zero_indices) > 1:
            canvas[yv, xv, zero_indices[1]] = fv
        canvas[yv, xv, 3] = alpha_u8

    def overlay_two_image(self, image, overlay):
        mask = overlay[:, :, 3] > 0
        overlay_alpha = overlay[mask, 3] / 255.0
        image_alpha = image[mask, 3] / 255.0 * (1 - overlay_alpha)

        new_alpha = overlay_alpha + image_alpha
        image[mask, :3] = (
            image[mask, :3] * image_alpha[..., np.newaxis] + overlay[mask, :3] * overlay_alpha[..., np.newaxis]
        ) / new_alpha[..., np.newaxis]
        image[mask, 3] = new_alpha * 255
        return image

    def _draw_trajectory_onto(
        self,
        canvas: NDArray,
        trajectory: list | np.ndarray,
        odom: tuple[NDArray, NDArray],
        traj_type: str,
    ) -> None:
        """Draw trajectory directly onto the provided canvas (no overlay)."""
        if len(trajectory) == 0:
            return

        position, _ = odom
        x, y, _ = position
        cur_x, cur_y = self.canvas.pose2coords((x, y))
        radius = int(self.config.map_radius / self.map_yml["resolution"])

        if traj_type == "robot":
            pixel_coords = self.calculate_pixel_coords_from_robot_traj(trajectory, cur_x, cur_y)
            color = self.config.robot_traj_color
            thickness = int(self.config.robot_traj_thickness)
        elif traj_type == "human":
            pixel_coords = self.calculate_pixel_coords_from_human_traj(trajectory, cur_x, cur_y)
            color = self.config.human_traj_color
            thickness = int(self.config.human_traj_thickness)
        else:
            raise ValueError("Invalid trajectory type")

        line_coords = self.calculate_in_bound_coordinates(radius, pixel_coords)

        # Draw on temporary canvas with reversed fading, dilate, invert fading back, then copy.
        # Reversed fading (new=255) ensures dilate MAX preserves z-order (newest on top).
        temp = np.zeros_like(canvas)
        self.line_to_image(temp, line_coords, color, thickness, timewise=True)
        if thickness > 1:
            # Compensate for diagonal thinning with larger elliptical kernel
            k = thickness + 2
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            cv2.dilate(temp, kernel, dst=temp)
        # Invert fading channels back to correct visual direction (old=bright, new=dark)
        zero_indices = np.where(np.array(color) == 0)[0]
        drawn = temp[:, :, 3] > 0
        for idx in zero_indices:
            temp[drawn, idx] = 255 - temp[drawn, idx]
        canvas[drawn] = temp[drawn]

    def calculate_arrow_end(self, map_array: NDArray, odom: tuple[NDArray, NDArray]) -> tuple[int, int, int, int]:
        """Calculate arrow end point from image center"""
        # Transform the pose to pixel coordinate
        position, orientation = odom

        # Extract the orientation angle in radians
        angle = self._convert_quaternion_to_yaw(orientation)

        # Calculate the arrow length and end point
        diff_x = self.config.local_arrow_length * np.cos(angle) / self.map_yml["resolution"]
        diff_y = -self.config.local_arrow_length * np.sin(angle) / self.map_yml["resolution"]
        # Convert to integer
        diff_x, diff_y = int(diff_x), int(diff_y)

        # convert the y coordinate to the image coordinate
        h, w, _ = map_array.shape
        x, y = w // 2, h // 2
        end_x = x + diff_x
        end_y = y + diff_y
        return x, y, end_x, end_y

    def _draw_arrow_onto(self, canvas: NDArray, odom: tuple[NDArray, NDArray], color: tuple) -> None:
        """Draw arrow directly onto the provided canvas (no overlay)."""
        x, y, end_x, end_y = self.calculate_arrow_end(canvas, odom)
        # Pre-scale alpha into color to avoid post-hoc full-image alpha scaling
        scaled_color = tuple(int(c * self.config.line_max_alpha) if i == 3 else c for i, c in enumerate(color))
        cv2.arrowedLine(
            canvas,
            (x, y),
            (end_x, end_y),
            scaled_color,
            int(self.config.local_arrow_thickness),
            line_type=self.config.line_type,
            shift=self.config.shift,
            tipLength=self.config.tip_length,
        )

    def calculate_pixel_coords_from_human_traj(self, human_trajectory: np.ndarray, cur_x, cur_y):
        """
        Calculate the pixel coordinates of the human trajectory

        Args:
            human_trajectory: The human trajectory, a numpy array of shape (N, 2) containing the pixel coordinates (y, x)
            cur_x: The current x pixel coordinate
            cur_y: The current y pixel coordinate
        Return:
            pixel_coords: The pixel coordinates of the human trajectory, around the current pose (cur_y, cur_x)
        """
        pixel_coords = human_trajectory - np.array([cur_y, cur_x])
        return pixel_coords

    def interpolate_trajectory(self, trajectory: list | np.ndarray) -> list[np.ndarray]:
        return _interpolate_trajectory(trajectory, self.config.distance_threshold)
