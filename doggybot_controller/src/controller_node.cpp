#include <doggybot_controller/controller_node.hpp>
#include <doggybot_controller/PP.hpp>
#include <doggybot_controller/PID.hpp>
namespace controller_node{

  controller::controller( const std::string& name ):
    Node(name){
    
    ros_subscriber = create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_scan/points", 10, std::bind(&controller::callback, this, std::placeholders::_1 ));
    ros_publisher = create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", 100);
    
    declare_parameter("control_mode", "");
    
    get_parameter("control_mode", ctrl_mode);
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
    start_time = std::chrono::steady_clock::now();
    if(flag == 0){
       cmd_vel_msg.linear.x = 0;
       cmd_vel_msg.angular.z = 0.1;
       ros_publisher->publish(cmd_vel_msg);
    
    }
   }
   void controller::controller_mode(geometry_msgs::msg::Twist &twist, double Px, double Py, double past_Px, double past_Py, double deltaT){
      if( ctrl_mode == "PP" ){
        pp ctrl;
        ctrl.PPcontroller(twist, Px, Py);
       }
      else if( ctrl_mode == "PID" ){
       pid ctrl;
       ctrl.PIDcontroller(twist, Px, Py, past_Px, past_Py, deltaT);
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
