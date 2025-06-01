from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Generate edumip description using xacro
    name = LaunchConfiguration('robot')
    robot_name_arg = DeclareLaunchArgument('robot', default_value="robot")
    doggy_description_pkg = FindPackageShare("doggybot_description")
   
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("doggybot_description"), "urdf", "doggybot.urdf.xacro"]),
            " ",
            
            
        ]
    )
    robot_description = {"robot_description": robot_description_content}  
    
    # Node for robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/joint_states", "/joint_states")]
    )
    
    robot_joint_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        parameters=[robot_description],
    )
    
   
   
    return LaunchDescription([
        robot_name_arg,
        robot_state_publisher_node,        
        #robot_joint_publisher_node, 

     
    ])
