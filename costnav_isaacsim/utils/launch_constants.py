"""Constants used by the Isaac Sim launcher."""

DEFAULT_PHYSICS_DT = 1.0 / 120.0
DEFAULT_RENDERING_DT = 1.0 / 30.0
WARMUP_STEPS = 100
DEFAULT_ROBOT_NAME = "nova_carter"
DEFAULT_USD_PATHS = {
    "nova_carter": "omniverse://10.50.2.21/Users/worv/costnav/Street_sidewalk.usd",
    "segway_e1": "omniverse://10.50.2.21/Users/worv/costnav/street_sidewalk_segwaye1_Corrected.usd",
}
DEFAULT_ROBOT_PRIM_PATHS = {
    "nova_carter": "/World/Nova_Carter_ROS/chassis_link",
    "segway_e1": "/World/Segway_E1_ROS2/base_link",
}
ROBOT_NAME_ALIASES = {
    "segway": "segway_e1",
    "segway-e1": "segway_e1",
    "segwaye1": "segway_e1",
}
