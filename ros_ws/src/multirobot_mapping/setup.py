# Setup script for the multirobot_mapping package

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'multirobot_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')), # Include launch files
        (os.path.join('share', package_name, 'config'), glob('config/*')), # Include config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),     # Include rviz files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'map_merger = multirobot_mapping.map_merger:main',      # Add map_merger script
            'swarm_ex = multirobot_mapping.stochastic_explorer:main',    # Add swarm_explorer script
            'swarm_ex2 = multirobot_mapping.deterministic_explorer:main',  # Add swarm_explorer2 script
        ],
    },
)
