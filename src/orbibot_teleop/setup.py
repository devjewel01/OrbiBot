from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'orbibot_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jewel',
    maintainer_email='jewel.nath@orbitax.com',
    description='Keyboard teleoperation package for OrbiBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_stop_node = orbibot_teleop.emergency_stop_node:main',
            'ps_controller_node = orbibot_teleop.ps_controller_node:main',
        ],
    },
)
