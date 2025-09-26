from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'path_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashwin',
    maintainer_email='ashwin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_smoother = path_following.path_smoother_node:main',
            'trajectory_generator = path_following.trajectory_generator_node:main',
            'follower_node = path_following.follower_node:main',
        ],
    },
)
