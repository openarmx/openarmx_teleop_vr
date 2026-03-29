# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
#
# To view a copy of this license, visit:
# http://creativecommons.org/licenses/by-nc-sa/4.0/
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

from setuptools import find_packages, setup

package_name = 'openarmx_teleop_vr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_vr.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='2233614988@qq.com',
    description='OpenArm teleoperation using VR controllers with IK solving',
    license='CC-BY-NC-SA-4.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'openarmx_teleop_vr_node = openarmx_teleop_vr.openarmx_teleop_vr_node:main'
        ],
    },
)
