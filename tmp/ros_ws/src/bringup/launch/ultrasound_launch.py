from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('bringup'),
        'config',
        'params.yaml'
        )
    node_front=Node(
        package = 'ultrasound_sensor',
        executable = 'ultrasound_sensor_node',
        name = 'node_front',
        parameters = [config]
    )
    node_sidefront=Node(
        package = 'ultrasound_sensor',
        executable = 'ultrasound_sensor_node',
        name = 'node_sidefront',
        parameters = [config]
    )
    node_siderear=Node(
        package = 'ultrasound_sensor',
        executable = 'ultrasound_sensor_node',
        name = 'node_siderear',
        parameters = [config]
    )
    ld.add_action(node_front)
    ld.add_action(node_sidefront)
    ld.add_action(node_siderear)
    return ld