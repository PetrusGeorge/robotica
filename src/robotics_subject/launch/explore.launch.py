import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_lite',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'robot_base_frame':      'base_footprint',
            'costmap_topic':         '/map',
            'costmap_updates_topic': '/map_updates',
            'costmap_global_frame':  'map',
            'visualize':             True,
            'planner_frequency':     0.5,
            'progress_timeout':      20.0,
            'min_frontier_size':     0.10,
            'potential_scale':       2.0,
            'orientation_scale':     10.5,
            'gain_scale':            1.0,
            'transform_tolerance':   2.0,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(explore_node)
    return ld


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()