from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'orbibot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jewel',
    maintainer_email='jewel.nath@orbitax.com',
    description='Mecanum wheel control package for OrbiBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_controller = orbibot_control.mecanum_controller:main',
            'kinematics_test = orbibot_control.kinematics_test:main',
            'system_test = orbibot_control.system_test:main',
        ],
    },
)
