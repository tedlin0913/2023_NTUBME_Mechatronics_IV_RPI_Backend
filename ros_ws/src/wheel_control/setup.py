from setuptools import find_packages, setup

package_name = 'wheel_control'

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
    maintainer='tedlin',
    maintainer_email='ptengineerlin@gmail.com',
    description='Wheel control node',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_control_node = wheel_control.wheel_control_node:main'
        ],
    },
)
