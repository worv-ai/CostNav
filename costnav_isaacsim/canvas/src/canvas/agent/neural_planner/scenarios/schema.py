import hashlib
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
from pydantic import BaseModel, ConfigDict, Field


class Scenario(BaseModel):
    # Field "model_type" has conflict with protected namespace "model_".
    # https://github.com/pydantic/pydantic/discussions/7121
    model_config = ConfigDict(protected_namespaces=())

    # Used by neural planner
    semantic_uuid: str = Field(..., description="unique identifier for the scenario")
    model_guideline: str = Field(..., description="guideline for the Neural Model")

    # TODO: Currently not used by neural planner (map is fixed via map_yaml config).
    # These fields are received from costnav_isaacsim and kept for future expansion
    # (e.g. dynamic map switching per scenario).
    map_name: str = Field("", description="name of the map")
    sketch_map_name: str = Field("", description="name of the sketch map")
    drive_map_name: str = Field("", description="name of the drive map")
    spawn_map_name: Any = Field(None)
    sketch_guideline: str = Field("")
    map_metadata: Any = Field(None)
    metadata: Any = Field(default_factory=dict)
    meta_flags: Any = Field(default_factory=dict)
    notes: str = Field("")

    @property
    def uuid(self):
        return hashlib.md5(self.semantic_uuid.encode(encoding="utf-8")).hexdigest()

    @classmethod
    def from_json(cls, path: Path):
        with open(path, "r", encoding="utf-8") as f:
            return cls.model_validate_json(f.read())

    def to_json(self, path: Path, indent: int = 4):
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.model_dump_json(indent=indent))


class AnnotatedScenario(BaseModel):
    scenario: Scenario
    annotation: Any = Field(None, description="trajectory annotation as np.ndarray, in (y, x) format")
    # TODO: composite fields not used in inference, kept for future expansion
    composite: Any = Field(None, description="composite image of the scenario, expecting PIL.Image")
    composite_path: Any = None

    @classmethod
    def from_disk(cls, path: Path):
        """Load AnnotatedScenario from disk."""
        scenario = Scenario.from_json(path.with_suffix(".json"))
        annotation = np.load(path.with_suffix(".npy")) if path.with_suffix(".npy").exists() else None
        return cls(
            scenario=scenario,
            annotation=annotation,
            composite=None,
            composite_path=path.with_suffix(".png") if path.with_suffix(".png").exists() else None,
        )

    def to_disk(self, path: Path):
        self.scenario.to_json(path.with_suffix(".json"), indent=4)
        if self.annotation is not None:
            np.save(path.with_suffix(".npy"), self.annotation)
        if self.composite is not None:
            self.composite.save(path.with_suffix(".png"))

    def load_composite(self):
        if self.composite is not None:
            return
        if self.composite_path is None:
            return
        self.composite = Image.open(self.composite_path)

    def get_annotation_image_path(self):
        return self.composite_path

    @property
    def uuid(self):
        return self.scenario.uuid
