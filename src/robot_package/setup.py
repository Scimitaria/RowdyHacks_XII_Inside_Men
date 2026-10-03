from setuptools import find_packages, setup

package_name = 'robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='levi',
    maintainer_email='levi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor = robot_package.motor_node:main',
            'imu_tracker = robot_package.imu_tracker_node:main',
            'oak_camera = robot_package.oak_camera_node:main',
            'camera_viewer = robot_package.camera_viewer_node:main',
        ],
    },
)
