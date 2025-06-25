from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'orbibot_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        # Scripts
        (os.path.join('share', package_name, 'scripts'),
            glob(os.path.join('scripts', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='Hardware interface for OrbiBot robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hardware_node = orbibot_hardware.hardware_node:main',
            'hardware_test = orbibot_hardware.hardware_test:main',
            'motor_test = orbibot_hardware.motor_test:main',
            'motor_enable_client = orbibot_hardware.motor_enable_client:main',
            'check_permissions = orbibot_hardware.check_permissions:main',
            'hardware_monitor = orbibot_hardware.hardware_monitor:main',
            'rosmaster_debug = orbibot_hardware.rosmaster_debug:main',
            'api_debug = orbibot_hardware.api_debug:main',
        ],
    },
)