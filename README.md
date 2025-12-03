# 4-DoF Robotic Arm (UR5 Conversion) with Gripper – ROS 2 Humble

### A custom 4-DoF robotic manipulator built by modifying the Universal Robots UR5.
I redesigned the kinematics, reduced DOF from 6 → 4, added a Robotiq-style gripper, 
and enabled complete URDF + Gazebo + RViz + ros2_control integration.

## Features
✔ Custom 4-DoF arm based on UR5  
✔ Fully updated URDF/XACRO  
✔ Integrated Robotiq HandE gripper  
✔ Gazebo simulation + camera sensor  
✔ RViz visualization  
✔ ros2_control (mock hardware & trajectories)  
✔ Launch files: view, spawn, test  

## Folder Structure
src/
 ├── ur5_4dof/  
 ├── robotiq_hande_description/  
 └── ur_description/

## How to Run
```bash
colcon build
source install/setup.bash
ros2 launch ur5_4dof display_4dof_with_gripper.launch.py

