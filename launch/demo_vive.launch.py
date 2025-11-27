from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os import path

# Sow we don't have to repeat for composable and non-composable versions.
# --configfile 
PARAMETERS = [
    {'driver_args': f'--v 100 --force-calibrate'},
    {'tracking_frame': 'teleop_world'},
    {'imu_topic': 'imu'},
    {'joy_topic': 'joy'},
    {'cfg_topic': 'cfg'},
    {'lighthouse_rate': 4.0}]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('namespace', default_value='',
                              description='Namespace for the non-TF topics'),
        DeclareLaunchArgument('foxbridge', default_value='false',
                              description='Launch a foxglove bridge')]

    # Non-composable launch (regular node)
    libsurvive_node = Node(
        package='libsurvive_ros2',
        executable='libsurvive_ros2_node',
        name='libsurvive_ros2_node',
        output='screen',
        arguments=['--ros-args', '--log-level', "error"],
        parameters=PARAMETERS
    )

    # For foxglove websocket bridge.
    foxbridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        condition=IfCondition(LaunchConfiguration('foxbridge')),
        parameters=[
            {'port': 8765},
        ],
        output='log')


    return LaunchDescription(
        arguments + [
            libsurvive_node,
            foxbridge_node,
        ])