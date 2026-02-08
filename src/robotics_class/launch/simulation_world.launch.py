import os

from ament_index_python.packages import get_package_share_path

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    robotics_class_path = get_package_share_path("robotics_class")
    world_gazebo_path = robotics_class_path / "worlds/arena_1.world"
    install_dir = get_package_prefix("robotics_class")
    worlds_models_path = os.path.join(robotics_class_path, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + \
            ':' + install_dir + '/share' + ':' + worlds_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + worlds_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + \
            ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='5.00', description='The x-component of the initial position (meters).')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='5.00', description='The y-component of the initial position (meters).')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.15', description='The z-component of the initial position (meters).')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.00', description='The roll angle of the initial position (radians).')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.00', description='The pitch angle of the initial position (radians).')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.00', description='The yaw angle of the initial position (radians).')

    world_gazebo_arg = DeclareLaunchArgument(name="world", default_value=str(world_gazebo_path), description="starts world for simulation")
    
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', LaunchConfiguration('world')],
        output='screen'
    )

    spawn_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="jetauto_spawner",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "jetauto",
            "-x", LaunchConfiguration('x_pose'), "-y", LaunchConfiguration('y_pose'), "-z", LaunchConfiguration('z_pose'),
            "-R", LaunchConfiguration('roll'), "-P", LaunchConfiguration('pitch'), "-Y", LaunchConfiguration('yaw')
        ]
    )

    ld = LaunchDescription()

    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(z_pose_arg)
    ld.add_action(roll_arg)
    ld.add_action(pitch_arg)
    ld.add_action(yaw_arg)

    ld.add_action(world_gazebo_arg)

    ld.add_action(gazebo_launch)

    ld.add_action(spawn_node)

    return ld

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
