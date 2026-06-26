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
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
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
            'goal_sel = multirobot_mapping.random_goal_selector:main',
            'goal_sel2 = multirobot_mapping.random_goal_selector2:main',
            'map_merger = multirobot_mapping.map_merger:main',
            'swarm_ex = multirobot_mapping.swarm_explorer:main',
        ],
    },
)
