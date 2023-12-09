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
    node_sidefront_right=Node(
        package = 'ultrasound_sensor',
        executable = 'ultrasound_sensor_node',
        name = 'node_sidefront_right',
        parameters = [config]
    )
    node_siderear_right=Node(
        package = 'ultrasound_sensor',
        executable = 'ultrasound_sensor_node',
        name = 'node_siderear_right',
        parameters = [config]
    )
    node_sidefront_left=Node(
        package = 'ultrasound_sensor',
        executable = 'ultrasound_sensor_node',
        name = 'node_sidefront_left',
        parameters = [config]
    )
    node_siderear_left=Node(
        package = 'ultrasound_sensor',
        executable = 'ultrasound_sensor_node',
        name = 'node_siderear_left',
        parameters = [config]
    )
    ld.add_action(node_front)
    ld.add_action(node_sidefront_right)
    ld.add_action(node_siderear_right)
    ld.add_action(node_sidefront_left)
    ld.add_action(node_siderear_left)
    return ld