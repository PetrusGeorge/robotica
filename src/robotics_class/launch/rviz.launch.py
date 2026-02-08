import os
from ament_index_python.packages import get_package_share_directory

from launch.events import Shutdown
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler

def generate_launch_description():
    robotics_class_path = get_package_share_directory('robotics_class')

    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(robotics_class_path, 'rviz', 'view.rviz'),
        description='Full path to the RVIZ config file to use')

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', ReplaceString(
            source_file=rviz_config_file,
            replacements={'robot_1/': '', 'robot_1': ''})],
        parameters=[{'use_sim_time': True}],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(start_rviz_cmd)

    return ld

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
