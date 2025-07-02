from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    control_mode = LaunchConfiguration('control_mode')
    control_mode_arg = DeclareLaunchArgument('control_mode', default_value='PP', 
    choices =['PP', 'PID', 'MPC'],
    description='Select suitable control mode for the robot.')
    
    doggy_description_pkg = FindPackageShare("doggybot_description")
    doggy_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([doggy_description_pkg, "/launch/doggy_description.launch.py"]),
 
    )
    doggybot_pkg = FindPackageShare("doggybot_gazebo")
    world_file = PathJoinSubstitution([doggybot_pkg, "worlds", "world.sdf"])
    gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
    launch_arguments={'gz_args': ['-r', '-v 4 ', world_file]}.items()
    )
    robot_controller_config = PathJoinSubstitution([doggybot_pkg, "config", "controller.yaml"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("doggybot_description"), "urdf", "doggybot.urdf.xacro"]),
            " ",
            "sim_controllers:=",
            robot_controller_config,
            
        ]
    )
    robot_description = {"robot_description": robot_description_content}  
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[("/joint_states", "/joint_states")]
    )
    # spawn a robot in gazebo
    spawn_robot = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[ '-name', 'doggy', 
        '-topic', 'robot_description', 
        '-x', '0.0',
        '-y', '0.0',
        '-z', '1.8' ],
    output='screen',
  )

    # open in rviz interface
    rviz_file = [doggybot_pkg , "/rviz/doggy.rviz"]
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_file] 
        )
     
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/wyman/model/doggy/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/lidar_scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist[ignition.msgs.Twist',
            '/world/wyman/dynamic_pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
        ],
       remappings=[
                   ('/world/wyman/model/doggy/joint_state', '/joint_states'),
                   ('/cmd_vel','/diff_drive_controller/cmd_vel_unstamped'),
                   ('/world/wyman/dynamic_pose/info','/pose/info'),
                    ],
        output='screen'
    ) 
     
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ robot_description, robot_controller_config],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    
    joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        remappings=[
            ("/robot_description", "/robot_description"),
        ],
    )
    
    diffdrive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        parameters=[robot_controller_config],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("~/cmd_vel_unstamped", "/cmd_vel"),
           
        ],
    )
   
    controller = Node(
        package="doggybot_controller",
        executable="control",
        name="controller",
        output="screen",
        parameters=[
        {"control_mode" : control_mode}
        ]
         )
    broadcaster = Node(
        package="doggybot_gazebo",
        executable="tf_node",
        name="broadcast",
        output="screen",
        parameters=[
            { "use_sim_time": True },
            
        ],
        )
    return LaunchDescription([  
        #doggy_description,
        control_mode_arg,
        broadcaster,
        robot_state_publisher_node,
        spawn_robot,      
        SetEnvironmentVariable(
        name = 'IGN_GAZEBO_RESOURCE_PATH',
        value = PathJoinSubstitution([doggy_description_pkg, ".."]) 
       ),
        gazebo_launch,
        bridge,
        rviz_node,
        #control_node,
        joint_state_controller,
        diffdrive_controller,
        controller
    ])
