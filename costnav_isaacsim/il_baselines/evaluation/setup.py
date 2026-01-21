"""Setup file for costnav_il_baselines package.

This package can be installed via:
    pip install -e .  (for development)
    pip install .     (for production)

For ROS2 colcon build, use the CMakeLists.txt and package.xml instead.
"""

from setuptools import setup, find_packages
import os

package_name = "costnav_il_baselines"

# Get the directory containing setup.py
here = os.path.abspath(os.path.dirname(__file__))

# Check if we're in a ROS2 colcon build environment
# In that case, include data_files for ament
is_ros2_build = os.environ.get("AMENT_PREFIX_PATH") is not None

data_files = []
if is_ros2_build:
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/configs",
            [
                "configs/vint_eval.yaml",
                "configs/robot_carter.yaml",
            ],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/vint_policy.launch.py",
            ],
        ),
    ]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    package_dir={
        "models": "models",
        "agents": "agents",
        "nodes": "nodes",
    },
    data_files=data_files,
    include_package_data=True,
    install_requires=[
        "setuptools",
        "torch>=2.0.0",
        "torchvision>=0.15.0",
        "efficientnet_pytorch>=0.7.1",
        "opencv-python",
        "pyyaml",
        "numpy",
        "pillow",
    ],
    extras_require={
        "ros2": [
            "rclpy",
            "cv_bridge",
        ],
    },
    zip_safe=True,
    maintainer="CostNav Authors",
    maintainer_email="cyjun0304@naver.com",
    description="Imitation Learning Baseline Evaluation for CostNav",
    license="MIT",
    python_requires=">=3.10",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vint_policy_node = nodes.vint_policy_node:main",
        ],
    },
)
