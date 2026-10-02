from setuptools import setup
import os
from glob import glob

package_name = 'motor_stub'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xrak',
    maintainer_email='xrak@todo.todo',
    description='4-wheel mecanum /cmd_vel bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_stub = motor_stub.cmd_vel_stub:main',
        ],
    },
)
