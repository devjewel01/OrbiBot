from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orbibot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Install config files  
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        # Install world files
        (os.path.join('share', package_name, 'worlds'),
         glob(os.path.join('worlds', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jewel',
    maintainer_email='jewel@example.com',
    description='Complete simulation environment for OrbiBot including Gazebo integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation_hardware_node = orbibot_simulation.simulation_hardware_node:main',
            'simulation_control_node = orbibot_simulation.simulation_control_node:main',
        ],
    },
)