# Doggybot-Project
A Doggy robot with four wheels, currently controlled by the `Differential Drive controller` on ROS2 Humble. This project includes an easy-to-understand ROS2 fundamental structure and normal elements like publisher/subscriber, ros2_tf, simple URDF, and configuration of the controller_manager. For mobile robotics studies, there are high-level packages 'Doggybot_control' and 'Doggybot_planner' (TBD) in this project, which include classical control and planning approaches such as PID, LQR, MPC, A*, and RRT.  

## Installing CasADi
In the MPC process, the optimizer CasADi is necessary. This package can be found at the link below. 
```bash
  https://github.com/casadi/casadi.git
```
 Solver used in MPC is IPOPT, which can be installed in Ubuntu (I am using Ubuntu 22.04) by
 ```bash
 sudo apt install coinor-libipopt-dev coinor-libipopt1v5
```
## Starting Ignition Gazebo simulation
To start the Ignition Gazebo and RViz2 by
```bash
ros2 launch doggybot_gazebo doggy_gazebo.launch.py
```
The target point is published on the /goal_pose topic, which you can set using the “2D Goal Pose” button in RViz’s top toolbar.
Currently, control_mode can be selected in the arguments when using `ros2 launch`. For example, start with the PID controller by
```bash
ros2 launch doggybot_gazebo doggy_gazebo.launch.py control_mode:=PID
```
