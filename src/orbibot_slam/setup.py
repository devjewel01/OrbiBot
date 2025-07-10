from setuptools import find_packages, setup

package_name = 'orbibot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/slam.rviz']),
        ('share/' + package_name + '/maps', []),
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
        ],
    },
)
