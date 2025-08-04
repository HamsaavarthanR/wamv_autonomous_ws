from setuptools import find_packages, setup

package_name = 'teleop_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hamsaavarthan Ravichandar',
    maintainer_email='hamsa.ravichandar@sargas.ai',
    description='Autonomous Stack for Wam-V',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = teleop_ctrl.teleop_keyboard:main',
        ],
    },
)
