import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    simulator_pkg_share = get_package_share_directory('scanner_sim')
    assembler_pkg_share = get_package_share_directory('scanner_assembler')

    scanner_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(simulator_pkg_share, 'launch', 'scanner_sim.launch.py')
        ),
        launch_arguments={
            'use_gui': 'false',
        }.items()
    )

    cloud_assembler = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(assembler_pkg_share, 'launch', 'scanner_assembler.launch.py')
        ),
        launch_arguments={
            'scan': 'scan',
            'cloud': 'cloud',
            'use_sim_time': 'true',
            'cloud_frame': 'lidar_lpx_t1',
            'cloud_assembly_in': 'cloud',
            'cloud_assembly_out': 'cloud_assembly',
            'cloud_assembly_frame': 'map',
            'cloud_assembly_buffer': '500',
        }.items()
    )

    scan_action_server = Node(
        package='scanner',
        executable='scan_action_server',
        name='scan_action_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        scanner_simulator,
        cloud_assembler,
        scan_action_server,
    ])