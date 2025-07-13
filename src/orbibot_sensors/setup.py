from setuptools import find_packages, setup

package_name = 'orbibot_sensors'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/lidar.launch.py']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sensors.launch.py']),
        ('share/' + package_name + '/config', ['config/rplidar_params.yaml']),
        ('share/' + package_name + '/config', ['config/realsense_camera_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jewel Nath',
    maintainer_email='jewel.nath@orbitax.com',
    description='Sensor integration package for OrbiBot - RPLidar A1 and Intel RealSense D435',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Removed sensor_fusion - using robot_localization instead
        ],
    },
)
