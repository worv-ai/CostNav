import copy
import functools
from pathlib import Path

import numpy as np
import yaml
from numpy.typing import NDArray
from PIL import Image
from pydantic import BaseModel, Field


class SketchMap(BaseModel):
    image_file: Path = Field(..., description="The image file of the map")
    map_yml: Path = Field(..., description="The yaml file of the map, which contains the resolution and origin")

    # to make the SketchMap hashable, to cache load_image and load_yaml
    # https://github.com/pydantic/pydantic/discussions/5159
    __hash__ = object.__hash__

    @classmethod
    def from_yaml(cls, yaml_path: str | Path) -> "SketchMap":
        """Load a SketchMap from a yaml file (e.g. maps/sidewalk.yaml).

        The yaml must contain an 'image' field with the image filename
        relative to the yaml's directory.
        """
        yaml_path = Path(yaml_path)
        data = yaml.safe_load(yaml_path.read_text())
        image_file = yaml_path.parent / data["image"]
        return cls(image_file=image_file, map_yml=yaml_path)

    @functools.cache
    def _load_image(self):
        return Image.open(self.image_file)

    @functools.cache
    def _load_yaml(self):
        return yaml.safe_load(self.map_yml.read_text())

    def load_image(self):
        return copy.deepcopy(self._load_image())

    def load_yaml(self):
        return copy.deepcopy(self._load_yaml())

    def coords2pose(self, coord: NDArray) -> NDArray:
        """Convert the pixel coordinates to pose
        Args:
            coord: The pixel coordinates (n, 2) in the order of y, x

        Returns:
            NDArray: The pose (x, y)
        """
        coord = np.array(coord)
        if coord.ndim == 1:
            coord = coord[None, :]
        assert coord.shape[1] == 2, f"coord should be (n, 2), but got {coord.shape}"
        map_image, map_yml = self._load_image(), self._load_yaml()
        w, h = map_image.size
        x = coord[:, 1] * map_yml["resolution"] + map_yml["origin"][0]
        y = (h - coord[:, 0]) * map_yml["resolution"] + map_yml["origin"][1]
        pose = np.stack([x, y], axis=1)
        if pose.shape[0] == 1:
            return pose[0]
        return pose

    def pose2coords(self, pose: NDArray) -> NDArray:
        """Convert the pose array to pixel coordinates array
        Args:
            pose_array: The pose array (n, 2) in the order of x, y

        Returns:
            NDArray: The pixel coordinates array (n, 2) in the order of integer x, y
        """
        pose = np.array(pose)
        if pose.ndim == 1:
            pose = pose[None, :]
        assert pose.shape[1] == 2, f"pose should be (n, 2), but got {pose.shape}"
        map_image, map_yml = self._load_image(), self._load_yaml()
        w, h = map_image.size

        pixel_x = (pose[:, 0] - map_yml["origin"][0]) / map_yml["resolution"]
        pixel_x = np.round(pixel_x)
        pixel_x = np.clip(pixel_x, 0, w - 1)
        pixel_y = h - (pose[:, 1] - map_yml["origin"][1]) / map_yml["resolution"]
        pixel_y = np.round(pixel_y)
        pixel_y = np.clip(pixel_y, 0, h - 1)
        pixel_coords = np.stack([pixel_x, pixel_y], axis=1).astype(int)
        if pixel_coords.shape[0] == 1:
            return pixel_coords[0]

        return pixel_coords
