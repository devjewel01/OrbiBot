from setuptools import find_packages, setup

package_name = 'orbibot_sensors'

setup(
    name=package_name,
    version='0.0.0',
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
    maintainer='jewel',
    maintainer_email='jewel.nath@orbitax.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_fusion = orbibot_sensors.sensor_fusion:main',
        ],
    },
)
