import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch Arguments
    use_gui = LaunchConfiguration('use_gui')
    robot_name = "scanner"

    use_gui_arg = DeclareLaunchArgument(
            'use_gui',
            default_value='true'
        )

    scanner_sim_path = os.path.join(
        get_package_share_directory('scanner_sim'))

    robot_description_path = os.path.join(
        get_package_share_directory('scanner_description'))

    xacro_file = os.path.join(robot_description_path,
                              'urdf',
                              'scanner.urdf.xacro')

    doc = xacro.process_file(xacro_file)

    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(scanner_sim_path, 'worlds'), ':' +
            str(Path(robot_description_path).parent.resolve())
            ]
        )

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='scanner_world',
                          description='Gz sim World'),
           ]
    )

    # if use Gazebo GUI
    gazebo_gui = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                condition=IfCondition(use_gui),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 4', ' -r'])
                ]
                
             )
    # if not use Gazebo GUI
    gazebo_no_gui = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                condition=UnlessCondition(use_gui),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'), '.sdf', ' -v 4', ' -r', ' -s'])
                ]
                
             )
    
    gz_spawn_entity = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=['-string', robot_desc,
                           '-x', '0.0',
                           '-y', '-3.0',
                           '-z', '0.05',
                           '-R', '0.0',
                           '-P', '0.0',
                           '-Y', '1.5707',
                           '-name', robot_name,
                           '-allow_renaming', 'false'],
            )
        ],
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_forward_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'forward_position_controller'],
        output='screen'
    )

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('scanner_description'),
        'config',
        'gz_bridge.yaml'
    )
    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output='screen'
    )

    rviz_config_file = os.path.join(scanner_sim_path, 'rviz', 'scanner_visualize.rviz')
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    return LaunchDescription([
        use_gui_arg,
        load_joint_state_controller,
        load_forward_position_controller,
        gazebo_resource_path,
        arguments,
        gazebo_gui,
        gazebo_no_gui,
        gz_bridge_node,
        node_robot_state_publisher, 
        gz_spawn_entity,
        rviz,
    ])