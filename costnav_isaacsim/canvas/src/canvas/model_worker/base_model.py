from pathlib import Path

import yaml
from pydantic import BaseModel


class BaseModelYamlJsonMixin:
    """
    BaseModel with helper methods for loading and saving to yaml/json format.
    """

    @classmethod
    def from_yaml(cls, path: Path):
        with open(path, "r", encoding="utf-8") as f:
            return cls(**yaml.safe_load(f))

    def to_yaml(self: BaseModel, path: Path):
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.model_dump(model="json"), f)

    @classmethod
    def from_json(cls, path: Path):
        with open(path, "r", encoding="utf-8") as f:
            return cls.model_validate_json(f.read())

    def to_json(self: BaseModel, path: Path, indent: int = 4, *args, **kwargs):
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.model_dump_json(indent=indent, *args, **kwargs))


class BaseModelWithYamlJsonFromTo(BaseModel, BaseModelYamlJsonMixin):
    pass
