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
    line_max_alpha: int = 0.7  # Max alpha value for the line
    human_traj_thickness: float = 3
    robot_traj_thickness: float = 5

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
        # Draw trajectories
        map_array = self._draw_trajectory(map_array, self.human_trajectory, odom, "human", use_overlay=not is_empty)
        map_array = self._draw_trajectory(map_array, self.robot_trajectory, odom, "robot")

        # Draw arrow in the middle of the cropped map
        map_array = self._draw_arrow(map_array, odom, self.config.current_color)

        return map_array

    def _crop_map(self, image: NDArray, odom: tuple[NDArray, NDArray]) -> np.ndarray:
        """
        Crops the map around the current robot pose. If the crop area extends
        beyond the map boundaries, the outside area is filled with a padding color.
        """
        h, w, c = image.shape
        radius = int(self.config.map_radius / self.map_yml["resolution"])
        padding_value = (127, 127, 127, 255)

        # Convert robot pose to pixel coordinates
        position, _ = odom
        x, y, _ = position
        pixel_x, pixel_y = self.canvas.pose2coords((x, y))
        pixel_x, pixel_y = int(pixel_x), int(pixel_y)

        # Create output canvas with padding color
        output_size = (radius * 2, radius * 2)
        cropped_map = np.full((output_size[0], output_size[1], c), padding_value, dtype=image.dtype)

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

    def line_to_image(self, image, line_coords, color, thickness, timewise=True):
        """
        Draw the line coordinates on the image

        Args:
            image: The image to draw the line
            line_coords: The line coordinates to draw in (y1, x1, y2, x2) format
            color: The color of the line
            timewise: If True, the line will be faded out over time
                    If False, the line will be faded in over time
        """
        canvas = np.zeros_like(image)
        color = np.array(color)
        # get 0 value indices
        indices = np.where(color == 0)[0]
        N = len(line_coords)
        for i in range(N):
            y1, x1, y2, x2 = line_coords[i]
            # Draw line
            # value is getting smaller over time
            # Because all the values are 255, it will be white
            fraction = (i + 1) / N
            if timewise:
                # For alpha=0.7, go from 255 down to 0.3*255
                start = 255
                end = 0
            else:
                # For alpha=0.7, go from 0.3*255 up to 255
                start = 0
                end = 255
            value = start + fraction * (end - start)

            # set alpha value
            color[indices] = int(value)
            canvas = cv2.line(
                canvas,
                (x1, y1),
                (x2, y2),
                color=tuple(color.tolist()),
                thickness=thickness,
                lineType=self.config.line_type,
            )

        return canvas

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

    def _draw_trajectory(
        self,
        image: NDArray,
        trajectory: list | np.ndarray,
        odom: tuple[NDArray, NDArray],
        traj_type: str,
        use_overlay: bool = True,
    ) -> NDArray:
        """Draw trajectory on the image."""
        # Check if the trajectory is empty, return the image if it is
        if len(trajectory) == 0:
            return image

        # Calculate current coordinates
        position, _ = odom
        x, y, _ = position
        cur_x, cur_y = self.canvas.pose2coords((x, y))

        # Calculate radius
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
        canvas = self.line_to_image(image, line_coords, color, thickness, timewise=True)
        canvas[..., 3] = self.config.line_max_alpha * canvas[..., 3]

        if use_overlay:
            # Overlay the trajectory on the map
            image = self.overlay_two_image(image, canvas)

            return image
        return canvas

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

    def _draw_arrow(self, map_array: NDArray, odom: tuple[NDArray, NDArray], color: tuple) -> NDArray:
        # Calculate the arrow end point
        x, y, end_x, end_y = self.calculate_arrow_end(map_array, odom)

        # Draw the arrow on the map
        arrow_image = np.zeros_like(map_array)
        cv2.arrowedLine(
            arrow_image,
            (x, y),
            (end_x, end_y),
            color,
            int(self.config.local_arrow_thickness),
            line_type=self.config.line_type,
            shift=self.config.shift,
            tipLength=self.config.tip_length,
        )
        arrow_image[..., 3] = self.config.line_max_alpha * arrow_image[..., 3]

        # Overlay the arrow on the map
        map_array = self.overlay_two_image(map_array, arrow_image)
        return map_array

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
