from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('robotics_class')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_params = LaunchConfiguration('amcl_params')

    declare_map_file = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'class_map.yaml'),
        description='Full path to map yaml file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_amcl_params = DeclareLaunchArgument(
        'amcl_params',
        default_value=os.path.join(pkg_share, 'params', 'amcl.yaml'),
        description='Full path to AMCL parameters file'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}
        ]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_params,
            {'use_sim_time': use_sim_time}
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']}
        ]
    )

    return LaunchDescription([
        declare_map_file,
        declare_use_sim_time,
        declare_amcl_params,
        map_server_node,
        amcl_node,
        lifecycle_manager
    ])
