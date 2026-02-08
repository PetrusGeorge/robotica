# robotics_class

This package contains the simulation and low-level core of the jetauto robot that will be used in the robotics discipline.

## How to install

1 - Make a workspace.

2 - Clone this repository in src folder.

3 - Go to the workspace root and install dependencies.

``` sh
  rosdep install --from-paths src
```
4 - Make sure to also install the gazebo packages.

``` sh
  sudo apt install ros-humble-gazebo-ros-pkgs
```
5 - Build your workspace.

``` sh
  colcon build
```

## How to Use

1 - Launch the description of the jet auto transformations.

``` sh
  ros2 launch robotics_class robot_description.launch.py
```

2 - Launch the simulation world.

``` sh
  ros2 launch robotics_class simulation_world.launch.py
```

3 - Launch the ekf.

``` sh
  ros2 launch robotics_class ekf.launch.py
```

4 - Launch the rviz.

``` sh
  ros2 launch robotics_class rviz.launch.py
```

5 - Launch the teleop.

``` sh
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=jetauto/cmd_vel
```

6 - Launch the goto (in case of passing coordinates).

``` sh
  ros2 topic pub -1 /jetauto/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}\"'

```

## Communication interfaces

### Topics

``` yaml
topic: /jetauto/cmd_vel
type: geometry_msgs:msg:Twist
I/O: input

topic: /jetauto/odometry/filtered
type: nav_msgs::msg::Odometry
I/O: output

topic: /jetauto/odometry/unfiltered
type: nav_msgs::msg::Odometry
I/O: output

topic: /jetauto/imu/data
type: sensor_msgs::msg::Imu
I/O: output

topic: /jetauto/lidar/scan
type: sensor_msgs::msg::LaserScan
I/O: output
```

**/jetauto/cmd_vel** is a topic that makes it possible to send a speed setpoint to jetauto.

**/jetauto/odometry/filtered** It is a topic where you can read the jetauto odometry information after the ekf filter.

**/jetauto/odometry/unfiltered** It is a topic where you can read the jetauto odometry information before the ekf filter.

**/jetauto/imu/data** is a topic where you can read the measurement information of the IMU sensor.

**/jetauto/lidar/scan** is a topic where the measurement information of the lidar sensor can be read.
