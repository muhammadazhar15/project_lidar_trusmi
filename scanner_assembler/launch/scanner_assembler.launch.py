from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    laser_in = LaunchConfiguration('scan')
    cloud_out = LaunchConfiguration('cloud')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cloud_frame = LaunchConfiguration('cloud_frame')

    assembler_in = LaunchConfiguration('cloud_assembly_in')
    assembler_frame = LaunchConfiguration('cloud_assembly_frame')
    assembler_buffer = LaunchConfiguration('cloud_assembly_buffer')


    laser_in_arg = DeclareLaunchArgument(
        'scan',
        default_value='scan',
        description='input topic laser'
    )

    cloud_out_arg = DeclareLaunchArgument(
        'cloud',
        default_value='cloud',
        description='output topic pointcloud'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true'
    )

    cloud_frame_arg = DeclareLaunchArgument(
        'cloud_frame',
        default_value='laser',
        description='target frame for pointcloud output'
    )

    assembler_in_arg = DeclareLaunchArgument(
        'cloud_assembly_in',
        default_value='cloud',
        description='input topic pointcloud for assembly'
    )

    assembler_frame_arg = DeclareLaunchArgument(
        'cloud_assembly_frame',
        default_value='map',
        description='fixed frame for assembly process'
    )

    assembler_buffer_arg = DeclareLaunchArgument(
        'cloud_assembly_buffer',
        default_value='1000',
        description='buffer size for assembly process'
    )

    # ── LaserScan → PointCloud2 → /cloud_in (untuk assembler) ─────────────────
    laserscan_to_pointcloud = Node(
        package='pointcloud_to_laserscan',
        executable='laserscan_to_pointcloud_node',
        name='laserscan_to_pointcloud',
        output='screen',
        parameters=[{
            'target_frame': cloud_frame,
            'use_sim_time': use_sim_time,
            'use_inf': True,
            'transform_tolerance': 0.01,
        }],
        remappings=[
            ('scan_in', laser_in),
            ('cloud', cloud_out),
        ],
    )

    pointcloud_assembler = Node(
        package='scanner_assembler',
        executable='assembler',
        name='assembler',
        output='screen',
        remappings=[
            ('/cloud_in', assembler_in),
        ],
        parameters=[
            {
                'fixed_frame': assembler_frame,
                'scan_buffer_size': assembler_buffer,
            }
        ]
    )

    return LaunchDescription([
        laser_in_arg,
        cloud_out_arg,
        use_sim_time_arg,
        cloud_frame_arg,
        assembler_in_arg,
        assembler_frame_arg,
        assembler_buffer_arg,
        laserscan_to_pointcloud,
        pointcloud_assembler,
    ])
