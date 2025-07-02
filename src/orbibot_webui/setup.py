from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'orbibot_webui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), 
            glob('orbibot_webui/templates/*')),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'flask>=2.0.0',
    ],
    zip_safe=True,
    maintainer='jewel',
    maintainer_email='jewel.nath@orbitax.com',
    description='Lightweight web UI for OrbiBot sensor monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_monitor = orbibot_webui.web_monitor:main',
        ],
    },
)