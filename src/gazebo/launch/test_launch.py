from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('gazebo')

    default_rviz_config_path = urdf_tutorial_path / 'config/rviz/robot.rviz'

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    use_sim_time_arg = DeclareLaunchArgument(name='use_sim_time', default_value='false',
                                            description='Flag to enable use_sim_time')


    log_level = DeclareLaunchArgument(
        name='log_level', 
        default_value='INFO', 
        choices=['DEBUG','INFO','WARN','ERROR','FATAL'],
        description='Flag to set log level')
    
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo'), '/launch/load.launch.py']),
           launch_arguments={
                'rvizconfig': [get_package_share_directory('gazebo'), '/config/rviz/mapping_cartographer.rviz'],
            }.items(),
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='own_log',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['-d', LaunchConfiguration('rvizconfig'), '--ros-args', '--log-level', LaunchConfiguration('log_level')]

    )


    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='log',
        remappings=[
            ('scan', 'scan'),
            ('imu', 'imu'),
        ],
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '-configuration_directory', [get_package_share_directory('gazebo'), '/config/nav/'],
            '-configuration_basename', 'cartographer.lua',
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    occupacy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='log',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '-resolution', '0.05', 
            '-publish_period_sec', '1.0',
            '--ros-args', '--log-level', LaunchConfiguration('log_level')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(log_level)
    ld.add_action(rviz_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_node)
    ld.add_action(cartographer)
    ld.add_action(occupacy_grid)

    return ld
