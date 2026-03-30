from setuptools import find_packages, setup

package_name = 'robot_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='labcesi2026',
    maintainer_email='labcesi2026@todo.todo',
    description='Differential drive motor control node',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'diff_drive_node = robot_drive.diff_drive_node:main',
            'keyboard_control = robot_drive.keyboard_control:main',
            'ir_sensor_node = robot_drive.ir_sensor_node:main'
        ],
    },
)
