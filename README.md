# Doggybot-Project
A Doggy robot with four wheels, currently controlled by the `Differential Drive controller` on ROS2 Humble. This project includes an easy-to-understand ROS2 fundamental structure and normal elements like publisher/subscriber, ros2_tf, simple URDF, and configuration of the controller_manager. For mobile robotics studies, there are high-level packages 'Doggybot_control' and 'Doggybot_planning' (TBD) in this project, which include classical control and planning approaches such as PID, LQR, MPC, A*, and RRT.  

## Installing CasADi
```bash
  https://github.com/casadi/casadi.git
```
To start the Ignition Gazebo and RViz2 by
```bash
ros2 launch doggybot_gazebo doggy_gazebo.launch.py
```
Put a box in the Gazebo environment attracting the robot to follow.

Currently, control_mode can be selected in the arguments when using `ros2 launch`. For example, start with the PID controller by
```bash
ros2 launch doggybot_gazebo doggy_gazebo.launch.py control_mode:=PID
```
