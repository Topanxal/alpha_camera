from setuptools import setup
import os
from glob import glob

package_name = 'alfa_robot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ar',
    maintainer_email='ar@todo.todo',
    description='Vision module for Logistics Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scene_analyzer = alfa_robot_vision.scene_analyzer_node:main',
            'visual_servo = alfa_robot_vision.visual_servo_node:main',
        ],
    },
)
