"""Simulate a Tello drone (offline model DB)"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'arena_world.world')
    urdf_path  = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    # (OPCIONAL) aponte caminhos locais se quiser:
    # arena_models = os.path.expanduser('~/arena_worlds/arena_models')
    # arena_worlds = os.path.expanduser('~/arena_worlds/worlds')

    return LaunchDescription([
        # 1) DESATIVAR banco de modelos online
        SetEnvironmentVariable(name='GAZEBO_MODEL_DATABASE_URI', value=''),

        # 2) (OPCIONAL) garantir que seus models/worlds locais estejam no path
        # SetEnvironmentVariable(name='GAZEBO_MODEL_PATH',
        #     value=f"{os.environ.get('GAZEBO_MODEL_PATH','')}:{arena_models}"),
        # SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH',
        #     value=f"{os.environ.get('GAZEBO_RESOURCE_PATH','')}:{arena_worlds}"),

        # Gazebo Classic + gazebo_ros
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ], output='screen'),

        # Spawn do Tello
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', '1', '0']),

        # TF
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', arguments=[urdf_path]),
    ])
