from setuptools import find_packages, setup

# Import libraries for configuration and lauhnch files setup
import os
from glob import glob

# Package name
package_name = 'slam_nav_stack'

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
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hamsaavarthan Ravichandnar',
    maintainer_email='hamsaavarthanr@gmail.com',
    description='SLAM and Autonomous Navigation Stack for Wam-V boat',
    license='Apache license-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = slam_nav_stack.test:main',
            'preprocess_lidar = slam_nav_stack.preprocess_lidar:main',
            'tf_camera_init = slam_nav_stack.tf_camera_init:main',
        ],
    },
)
