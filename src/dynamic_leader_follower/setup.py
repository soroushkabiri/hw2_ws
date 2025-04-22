from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
package_name = 'dynamic_leader_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soroushkabiri92',
    maintainer_email='soroushkabiri92@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['turtle_tf2_broadcaster = dynamic_leader_follower.turtle_tf2_broadcaster:main',
        'turtle_tf2_listener = dynamic_leader_follower.turtle_tf2_listener:main',
        'turtle_tf2_listener_random_leader = dynamic_leader_follower.turtle_tf2_listener_random_leader:main',

        ],
    },
)
