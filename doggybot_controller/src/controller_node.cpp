#include <doggybot_controller/controller_node.hpp>
#include <doggybot_controller/PP.hpp>
#include <doggybot_controller/PID.hpp>
#include <doggybot_controller/MPC.hpp>
#include <iostream>

namespace controller_node{

  controller::controller( const std::string& name ):
    Node(name){
    
    group1 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group2 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    group3 = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = group1;

    rclcpp::SubscriptionOptions options2;
    options2.callback_group = group2;
    
    rclcpp::SubscriptionOptions options3;
    options3.callback_group = group3;
    
    ros_subscriber = create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_scan/points", 100, std::bind(&controller::callback, this, std::placeholders::_1 ), options1);
    goal_pose_subscriber = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 100, std::bind(&controller::goal_pose_callback, this, std::placeholders::_1 ), options2);
    robot_pose_subscriber = create_subscription<geometry_msgs::msg::PoseArray>("/pose/info", 100, std::bind(&controller::robot_pose_callback, this, std::placeholders::_1 ), options3);
   
    ros_publisher = create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", 100);
    
    declare_parameter("control_mode", "");
    
    get_parameter("control_mode", ctrl_mode);
    
    if      (ctrl_mode == "MPC") mode = Mode::MPC;
    else if (ctrl_mode == "PP")  mode = Mode::PP;
    else if (ctrl_mode == "PID") mode = Mode::PID;
  }
  
  void controller::callback( const sensor_msgs::msg::PointCloud2::SharedPtr msg ){ 
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);
    geometry_msgs::msg::Twist cmd_vel_msg;
    int flag = 0;
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed = end_time - start_time;
    double time = std::chrono::duration<double>(elapsed).count(); 
    /*
    // Print every point
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      const auto &pt = cloud.points[i];
      if (!std::isfinite(pt.x) ||
          !std::isfinite(pt.y) ||
          !std::isfinite(pt.z) 
          )
      {
        continue;  // skip NaNs
      }
      if(pt.z>-0.3)
       {
        flag = 1;
        
        controller_mode(cmd_vel_msg, pt.x, pt.y, past_x, past_y,  time);
        past_x = pt.x;
        past_y = pt.y; 
        ros_publisher->publish(cmd_vel_msg);
        break;
       
       }
    }
    
    if(flag == 0){
       cmd_vel_msg.linear.x = 0;
       cmd_vel_msg.angular.z = 0.1;
       ros_publisher->publish(cmd_vel_msg);
    std::cout<<"/////"<<goal_pos.linear.x<<" "<<goal_pos.linear.y<<" "<<goal_pos.angular.z<<" "<<std::endl;
    std::cout<<"/////"<<robot_pos.linear.x<<" "<<robot_pos.linear.y<<" "<<robot_pos.angular.z<<" "<<std::endl;
    
    }
    */
    geometry_msgs::msg::Twist goal2robot;
    goal2robot.linear.x = cos(robot_pos.angular.z)*(goal_pos.linear.x - robot_pos.linear.x) + sin(robot_pos.angular.z)*(goal_pos.linear.y - robot_pos.linear.y);
    goal2robot.linear.y = cos(robot_pos.angular.z)*(goal_pos.linear.y - robot_pos.linear.y) + sin(robot_pos.angular.z)*(robot_pos.linear.x - goal_pos.linear.x);
   // std::cout<<"/////"<< goal2robot.linear.x<<" "<< goal2robot.linear.y<<" "<<std::endl;
    
    if(sqrt(goal2robot.linear.x*goal2robot.linear.x + goal2robot.linear.y*goal2robot.linear.y)>0.3)
 
    {
       controller_mode(cmd_vel_msg, goal2robot.linear.x, goal2robot.linear.y, past_x, past_y,  time);
        past_x = goal2robot.linear.x;
        past_y = goal2robot.linear.y; 
        
        ros_publisher->publish(cmd_vel_msg);
       
    }
   
    start_time = std::chrono::steady_clock::now();
   
   }
   void controller::controller_mode(geometry_msgs::msg::Twist &twist, double Px, double Py, double past_Px, double past_Py, double deltaT){
      switch(mode){
      case Mode::MPC:
      {
        std::vector<double> state(3, 0);
        std::vector<double> target(3, 0);

        state[0] = 0;
        state[1] = 0;
        state[2] = 0;
        target[0] = Px;
        target[1] = Py;
        target[2] = 0;
        if(target[0]>0.0){
        mpc ctrl;
        ctrl.MPCcontroller(twist, state, target, deltaT);
        }
        else{
            twist.linear.x = 0;
            twist.angular.z = 0.2;
        }
        break;
      }
      case Mode::PP:
      {
      
        pp ctrl;
        ctrl.PPcontroller(twist, Px, atan2(Py,Px));
        break;
       }
      case Mode::PID:
      {
       pid ctrl;
       ctrl.PIDcontroller(twist, Px, Py, past_Px, past_Py, deltaT);
       break;
      }
     }
    }
     
     void controller::goal_pose_callback( const geometry_msgs::msg::PoseStamped::SharedPtr msg ){
     double qx, qy, qz, qw;
     qx =  msg->pose.orientation.x;
     qy =  msg->pose.orientation.y;
     qz =  msg->pose.orientation.z;
     qw =  msg->pose.orientation.w;
     double siny_cosp = 2*(qw*qz - qx*qy);
     double cosy_cosp = 1 - 2*(qy*qy + qz*qz);
     double yaw   = std::atan2(siny_cosp, cosy_cosp);
     
     
     goal_pos.linear.x = msg->pose.position.x;
     goal_pos.linear.y = msg->pose.position.y;
     goal_pos.angular.z = yaw;
    // std::cout<<"/////"<<goal_pos.linear.x<<" "<<goal_pos.linear.y<<" "<<goal_pos.angular.z<<" "<<std::endl;
   
   }
   void controller::robot_pose_callback( const geometry_msgs::msg::PoseArray::SharedPtr msg ){

    //std::cout<<msg->poses.size()<<std::endl;
    for(int i=0; i<msg->poses.size(); i++){
      if( msg->poses[i].position.z>0.6 
       && msg->poses[i].position.z<0.8){
       double qx, qy, qz, qw;
       qx =  msg->poses[i].orientation.x;
       qy =  msg->poses[i].orientation.y;
       qz =  msg->poses[i].orientation.z;
       qw =  msg->poses[i].orientation.w;
       double siny_cosp = 2*(qw*qz - qx*qy);
       double cosy_cosp = 1 - 2*(qy*qy + qz*qz);
       double yaw   = std::atan2(siny_cosp, cosy_cosp);
       
       robot_pos.linear.x = msg->poses[i].position.x;
       robot_pos.linear.y = msg->poses[i].position.y;
       robot_pos.angular.z = yaw;
       return;
       }
    }
   }
     
   }
   
   
   
   
  int main( int argc, char** argv ){

  rclcpp::init( argc, argv );
  
  auto node = std::make_shared<controller_node::controller>("sub");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  
  return 0;

}
