from setuptools import find_packages, setup

package_name = 'orbibot_localization'

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
    maintainer='Jewel Nath',
    maintainer_email='jewel.nath@orbitax.com',
    description='Enhanced localization package for OrbiBot with sensor fusion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_localization = orbibot_localization.ekf_localization:main',
            'sensor_fusion = orbibot_localization.sensor_fusion:main',
        ],
    },
)
