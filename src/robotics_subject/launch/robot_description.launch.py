import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription, LaunchService
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)

    robotics_subject_path = get_package_share_directory('robotics_subject')
    urdf_path = os.path.join(robotics_subject_path, 'urdf/jetauto.xacro')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    ld = LaunchDescription()

    ld.add_action(use_sim_time_arg)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    return ld

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()