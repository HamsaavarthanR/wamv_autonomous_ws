from setuptools import find_packages, setup

# IMport libraries for configuration and lauhnch files setup
import os
from glob import glob

package_name = 'perception_stack'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Model files
        (os.path.join('share', package_name, 'my_model'), glob('my_model/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hamsaavarthan Ravichandar',
    maintainer_email='hamsaavarthanr@gmail.com',
    description='Perception Stack for Wam-V',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_detector = perception_stack.obstacle_detection:main',
            'sensor_fusion = perception_stack.sensor_fusion:main',
        ],
    },
)
