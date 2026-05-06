from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'openarmx_teleop_vr'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenArmX Team',
    maintainer_email='your_email@example.com',
    description='VR teleoperation for OpenArmX using Pinocchio IK solver',
    license='MIT',
    entry_points={
        'console_scripts': [
            'openarmx_teleop_vr_node = openarmx_teleop_vr.openarmx_teleop_vr_node:main',
        ],
    },
    scripts=[],
)
