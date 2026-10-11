from glob import glob

from setuptools import find_packages, setup

package_name = 'diffbot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abeera',
    maintainer_email='afarooq05@gmail.com',
    description='Hand-written PID controllers for a differential-drive robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = diffbot_control.wall_follower_node:main',
            'waypoint_follower = diffbot_control.waypoint_follower_node:main',
            'trajectory_recorder = diffbot_control.trajectory_recorder_node:main',
        ],
    },
)
