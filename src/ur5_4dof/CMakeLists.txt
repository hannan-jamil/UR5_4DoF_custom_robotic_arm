cmake_minimum_required(VERSION 3.5)
project(ur5_4dof)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

find_package(rviz2 REQUIRED)
find_package(gazebo_ros REQUIRED)                 # ← add
find_package(ur_description REQUIRED)
find_package(robotiq_hande_description REQUIRED)

install(
  DIRECTORY
    urdf
    meshes
    launch
    worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

