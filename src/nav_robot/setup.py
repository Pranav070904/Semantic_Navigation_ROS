from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'nav_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        (os.path.join('share',package_name,'config'),glob('config/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
            
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pranav',
    maintainer_email='pranav@todo.todo',
    description='Navigatoin robot package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grounding_dino_node=nav_robot.grounding_dino_node:main',
            'navigation_node=nav_robot.navigation_node:main',
            'reasoning_node=nav_robot.reasoning_node:main',
            'orch_node=nav_robot.orch_node:main',
            'nav2_goal_node= nav_robot.nav2_goal_node:main',
            'local_search_node=nav_robot.local_search_node:main',
        ],
    },
)
