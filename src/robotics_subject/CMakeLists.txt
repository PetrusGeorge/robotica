cmake_minimum_required(VERSION 3.8)
project(robotics_subject)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(gazebo_mecanum_drive SHARED
  src/gazebo_mecanum_drive.cpp)

add_library(gazebo_mecanum_pid_drive SHARED
  src/gazebo_mecanum_pid_drive.cpp)

target_include_directories(gazebo_mecanum_drive PUBLIC include)
target_include_directories(gazebo_mecanum_pid_drive PUBLIC include)

ament_target_dependencies(gazebo_mecanum_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

ament_target_dependencies(gazebo_mecanum_pid_drive
  "gazebo_dev"
  "gazebo_ros"
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

install(DIRECTORY
  launch
  urdf
  models
  rviz
  worlds
  params
  include/
	DESTINATION share/${PROJECT_NAME}
)

# Instala o script Python e marca como executável
install(PROGRAMS scripts/maze_solver_nav2_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME maze_solver_nav2_node
)

# Instala o script Python e marca como executável
install(PROGRAMS scripts/color_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME color_detector_node
)

install(PROGRAMS scripts/maze_solver_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME maze_solver_node
)

install(TARGETS
        gazebo_mecanum_drive
        gazebo_mecanum_pid_drive
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
