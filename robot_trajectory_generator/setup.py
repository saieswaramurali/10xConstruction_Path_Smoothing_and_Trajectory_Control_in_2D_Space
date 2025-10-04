from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot_trajectory_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sai',
    maintainer_email='sai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_collector = robot_trajectory_generator.waypoint_collector:main',
            'trajectory_generator_example = robot_trajectory_generator.trajectory_generator_example:main',
            'test_case_publisher = robot_trajectory_generator.test_case_publisher:main',
            'catmull_rom_generator = robot_trajectory_generator.catmull_rom_generator:main',
            'velocity_profiler = robot_trajectory_generator.velocity_profiler:main',
            'pure_pursuit_controller = robot_trajectory_generator.pure_pursuit_controller:main',
        ],
    },
)
