from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odrive_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='todo@todo.com',
    description='Standalone ROS2 node to control ODrive via CAN using joystick input',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odrive_control = odrive_control.odrive_control:main',
            'joy_to_control_message = odrive_control.joy_to_control_message:main',
        ],
    },
)
