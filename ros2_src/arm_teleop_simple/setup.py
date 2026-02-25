from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_teleop_simple'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_user',
    maintainer_email='adpoitras@gmail.com',
    description='Simple planar 2-link inverse kinematics teleop for CANopen motors',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'planar_ik_teleop = arm_teleop_simple.planar_ik_teleop:main',
        ],
    },
)
