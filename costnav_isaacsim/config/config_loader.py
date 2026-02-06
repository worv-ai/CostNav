"""Configuration loader for CostNav mission settings."""

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import yaml

logger = logging.getLogger("costnav_config")

# Default config file path (relative to this module)
DEFAULT_CONFIG_PATH = Path(__file__).parent / "mission_config.yaml"


@dataclass
class MarkerConfig:
    """RViz marker configuration."""

    enabled: bool = True
    start_topic: str = "/start_marker"
    goal_topic: str = "/goal_marker"
    robot_topic: str = "/robot_marker"
    arrow_length: float = 1.0
    arrow_width: float = 0.2
    arrow_height: float = 0.2
    robot_length: float = 0.9
    robot_width: float = 0.5
    robot_height: float = 0.2


@dataclass
class Nav2Config:
    """Nav2 integration configuration."""

    wait_time: float = 10.0
    initial_pose_delay: float = 1.0
    initial_pose_topic: str = "/initialpose"
    goal_pose_topic: str = "/goal_pose"
    odom_topic: str = "/odom"


@dataclass
class TeleportConfig:
    """Robot teleportation configuration."""

    height_offset: float = 0.5
    robot_prim: str = "/World/Nova_Carter_ROS/chassis_link"


@dataclass
class SamplingConfig:
    """Position sampling configuration."""

    max_attempts: int = 100
    validate_path: bool = True
    edge_margin: float = 0.5  # Minimum distance from navmesh edges (meters)


def _get_omni_url() -> str:
    """Get OMNI_URL from environment, defaulting to localhost."""
    import os
    return os.environ.get("OMNI_URL", "omniverse://localhost")


@dataclass
class FoodConfig:
    """Food evaluation configuration for spoilage tracking."""

    enabled: bool = False  # Whether food evaluation is enabled
    usd_path: str = f"{_get_omni_url()}/Users/worv/costnav/foods/popcorn/popcorn.usd"
    prim_path: str = "/World/Food"  # Where the food USD is referenced in the stage
    pieces_prim_path: str = "Popcorn/PopcornBucket/PopcornPieces"  # Relative path to pieces under food prim
    bucket_prim_path: str = "Popcorn/PopcornBucket"  # Relative path to bucket/container under food prim
    spoilage_threshold: float = 0.05  # Fraction of pieces that can be lost (0.0 = no loss allowed)


@dataclass
class InjuryCostConfig:
    mais_0: float = 0.0
    mais_1: float = 0.0
    mais_2: float = 0.0
    mais_3: float = 0.0
    mais_4: float = 0.0
    mais_5: float = 0.0
    fatality: float = 0.0

    def as_dict(self) -> dict:
        return {
            "mais_0": self.mais_0,
            "mais_1": self.mais_1,
            "mais_2": self.mais_2,
            "mais_3": self.mais_3,
            "mais_4": self.mais_4,
            "mais_5": self.mais_5,
            "fatality": self.fatality,
        }


@dataclass
class InjuryConfig:
    enabled: bool = True
    method: str = "delta_v"
    crash_mode: str = "all"
    robot_mass: float = 50.0  # Robot mass in kg for delta-v = impulse / mass calculation
    robot_height: Optional[float] = None
    pedestrian_height: Optional[float] = None
    costs: InjuryCostConfig = field(default_factory=InjuryCostConfig)


@dataclass
class MissionConfig:
    """Complete mission configuration."""

    # Mission execution
    timeout: float = 3600.0  # 1 hour
    goal_tolerance: float = 1.0  # Distance to goal for success (meters)

    # Distance constraints
    min_distance: float = 5.0
    max_distance: float = 50.0

    # Sub-configurations
    nav2: Nav2Config = field(default_factory=Nav2Config)
    teleport: TeleportConfig = field(default_factory=TeleportConfig)
    markers: MarkerConfig = field(default_factory=MarkerConfig)
    sampling: SamplingConfig = field(default_factory=SamplingConfig)
    food: FoodConfig = field(default_factory=FoodConfig)
    injury: InjuryConfig = field(default_factory=InjuryConfig)

    @classmethod
    def from_dict(cls, data: dict) -> "MissionConfig":
        """Create MissionConfig from dictionary (parsed YAML)."""
        mission_data = data.get("mission", {})
        distance_data = data.get("distance", {})
        nav2_data = data.get("nav2", {})
        teleport_data = data.get("teleport", {})
        markers_data = data.get("markers", {})
        sampling_data = data.get("sampling", {})
        food_data = data.get("food", {})
        injury_data = data.get("injury", {})

        # Parse Nav2 config
        nav2_topics = nav2_data.get("topics", {})
        nav2_config = Nav2Config(
            wait_time=nav2_data.get("wait_time", 10.0),
            initial_pose_delay=nav2_data.get("initial_pose_delay", 1.0),
            initial_pose_topic=nav2_topics.get("initial_pose", "/initialpose"),
            goal_pose_topic=nav2_topics.get("goal_pose", "/goal_pose"),
            odom_topic=nav2_topics.get("odom", "/odom"),
        )

        # Parse teleport config
        teleport_config = TeleportConfig(
            height_offset=teleport_data.get("height_offset", 0.5),
            robot_prim=teleport_data.get("robot_prim", "/World/Nova_Carter_ROS/chassis_link"),
        )

        # Parse markers config
        marker_topics = markers_data.get("topics", {})
        marker_scale = markers_data.get("scale", {})
        robot_scale = markers_data.get("robot_scale", {})
        markers_config = MarkerConfig(
            enabled=markers_data.get("enabled", True),
            start_topic=marker_topics.get("start", "/start_marker"),
            goal_topic=marker_topics.get("goal", "/goal_marker"),
            robot_topic=marker_topics.get("robot", "/robot_marker"),
            arrow_length=marker_scale.get("arrow_length", 1.0),
            arrow_width=marker_scale.get("arrow_width", 0.2),
            arrow_height=marker_scale.get("arrow_height", 0.2),
            robot_length=robot_scale.get("length", 0.9),
            robot_width=robot_scale.get("width", 0.5),
            robot_height=robot_scale.get("height", 0.2),
        )

        # Parse sampling config
        sampling_config = SamplingConfig(
            max_attempts=sampling_data.get("max_attempts", 100),
            validate_path=sampling_data.get("validate_path", True),
            edge_margin=sampling_data.get("edge_margin", 0.5),
        )

        # Parse food config
        food_config = FoodConfig(
            enabled=food_data.get("enabled", False),
            usd_path=food_data.get("usd_path", ""),
            prim_path=food_data.get("prim_path", "/World/Food"),
            pieces_prim_path=food_data.get("pieces_prim_path", "PopcornBucket/PopcornPieces"),
            bucket_prim_path=food_data.get("bucket_prim_path", "PopcornBucket"),
            spoilage_threshold=food_data.get("spoilage_threshold", 0.0),
        )

        injury_costs_data = injury_data.get("costs", {})
        injury_costs = InjuryCostConfig(
            mais_0=injury_costs_data.get("mais_0", 0.0),
            mais_1=injury_costs_data.get("mais_1", 0.0),
            mais_2=injury_costs_data.get("mais_2", 0.0),
            mais_3=injury_costs_data.get("mais_3", 0.0),
            mais_4=injury_costs_data.get("mais_4", 0.0),
            mais_5=injury_costs_data.get("mais_5", 0.0),
            fatality=injury_costs_data.get("fatality", 0.0),
        )
        injury_config = InjuryConfig(
            enabled=injury_data.get("enabled", True),
            method=injury_data.get("method", "delta_v"),
            crash_mode=injury_data.get("crash_mode", "all"),
            robot_mass=injury_data.get("robot_mass", 50.0),
            robot_height=injury_data.get("robot_height"),
            pedestrian_height=injury_data.get("pedestrian_height"),
            costs=injury_costs,
        )

        return cls(
            timeout=mission_data.get("timeout", 3600.0),
            goal_tolerance=mission_data.get("goal_tolerance", 1.0),
            min_distance=distance_data.get("min", 5.0),
            max_distance=distance_data.get("max", 50.0),
            nav2=nav2_config,
            teleport=teleport_config,
            markers=markers_config,
            sampling=sampling_config,
            food=food_config,
            injury=injury_config,
        )

    def to_dict(self) -> dict:
        """Convert to dictionary for legacy compatibility."""
        return {
            "mission_timeout": self.timeout,
            "min_distance": self.min_distance,
            "max_distance": self.max_distance,
            "nav2_wait": self.nav2.wait_time,
        }


def load_mission_config(config_path: Optional[str] = None) -> MissionConfig:
    """Load mission configuration from YAML file.

    Args:
        config_path: Path to config file. If None, uses default config.

    Returns:
        MissionConfig instance with loaded settings.
    """
    path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH

    if not path.exists():
        logger.warning(f"Config file not found: {path}. Using defaults.")
        return MissionConfig()

    try:
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
        logger.info(f"Loaded mission config from: {path}")
        return MissionConfig.from_dict(data)
    except Exception as e:
        logger.error(f"Failed to load config: {e}. Using defaults.")
        return MissionConfig()
