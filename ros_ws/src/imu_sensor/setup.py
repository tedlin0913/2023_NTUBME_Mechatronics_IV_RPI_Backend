from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'imu_sensor'

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
    maintainer='Ted Lin',
    maintainer_email='ptengineerlin@gmail.com',
    description='Imu sensor node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'imu_sensor_node = imu_sensor.imu_sensor_node:main'
        ],
    },
)
