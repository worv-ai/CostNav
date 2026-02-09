"""Setup file for costnav_il_baselines package.

This package can be installed via:
    pip install -e .  (for development)
    pip install .     (for production)
"""

from setuptools import find_packages, setup

package_name = "costnav_il_baselines"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    include_package_data=True,
    install_requires=[
        "setuptools",
        "torch>=2.0.0",
        "torchvision>=0.15.0",
        "efficientnet_pytorch>=0.7.1",
        "opencv-python",
        "pyyaml",
        "numpy>=1.24.0,<2",  # cv_bridge from ROS2 requires numpy<2
        "pillow",
        "rclpy",
        "cv_bridge",
    ],
    zip_safe=True,
    maintainer="CostNav Authors",
    maintainer_email="cyjun0304@naver.com",
    description="Imitation Learning Baseline Evaluation for CostNav",
    license="MIT",
    python_requires=">=3.10",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vint_policy_node = evaluation.nodes.vint_policy_node:main",
        ],
    },
)
