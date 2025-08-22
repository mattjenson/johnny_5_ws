from setuptools import setup
from glob import glob
import os

package_name = 'johnny_5_base_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ── ROS 2 resource index ─────────────────────────────
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # ── Package manifest ────────────────────────────────
        ('share/' + package_name,
         ['package.xml']),
        # ── Launch files (✅ add this!) ─────────────────────
        ('share/' + package_name + '/launch',
         ['launch/base_bringup.launch.py']),
    ],
    install_requires=['setuptools', 'rclpy', 'lgpio'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='matt.jenson@gmail.com',
    description='Python motor driver for Johnny 5 tracked base',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_driver = johnny_5_base_driver.base_driver:main',
        ],
    },
)