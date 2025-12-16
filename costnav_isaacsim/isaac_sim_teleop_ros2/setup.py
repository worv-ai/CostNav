from setuptools import setup

package_name = 'isaac_sim_teleop_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Haebin Seong',
    maintainer_email='hbseong97@gmail.com',
    description='Provides teleoperation using joy for isaac_sim with ROS2',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

