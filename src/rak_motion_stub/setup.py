from setuptools import setup

package_name = 'rak_motion_stub'

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
    description='Test-only motion relay and simulated base telemetry adapter.',
    license='MIT',
    # [AI] tests_require 在现代 setuptools 失效；用 extras_require["test"] 让 colcon 走 pytest。
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cmd_vel_stub = rak_motion_stub.cmd_vel_stub:main',
            'cmd_vel_sim = rak_motion_stub.cmd_vel_sim:main',
            'cmd_vel_omni = rak_motion_stub.cmd_vel_omni:main',
        ],
    },
)
