#include <doggybot_controller/controller.hpp>
namespace rsp{

  subscriber::subscriber( const std::string& name ):
    Node(name){
    
    ros_subscriber = create_subscription<sensor_msgs::msg::PointCloud2>("/lidar_scan/points", 10, std::bind(&subscriber::callback, this, std::placeholders::_1 ));
    ros_publisher = create_publisher<geometry_msgs::msg::Twist>("/diff_drive_controller/cmd_vel_unstamped", 100);
  }
  
  void subscriber::callback( const sensor_msgs::msg::PointCloud2::SharedPtr msg ){ 
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_pc2, cloud);

    // Print every point
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      const auto &pt = cloud.points[i];
      if (!std::isfinite(pt.x) ||
          !std::isfinite(pt.y) ||
          !std::isfinite(pt.z) ||
          pt.z<-0.3
          )
      {
        continue;  // skip NaNs
      }
      if(pt.z>-0.3)
       {
        geometry_msgs::msg::Twist cmd_vel_msg;
        //pp::PPcontroller(cmd_vel_msg, pt.x, pt.y);
        if(abs(pt.y)>=0.7)
        cmd_vel_msg.angular.z = 0.2*pt.y;
        else{
        cmd_vel_msg.linear.x = 0.3*pt.x;
        cmd_vel_msg.angular.z = 0.05*pt.y;
        }
        
        
        ros_publisher->publish(cmd_vel_msg);
        break;
       
       }
      std::cout << "Point " << i
                << ": ["
                << pt.x << ", "
                << pt.y << ", "
                << pt.z << "]\n";
    }
   }
   }
  int main( int argc, char** argv ){

  rclcpp::init( argc, argv );
  
  auto node = std::make_shared<rsp::subscriber>("sub");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  
  return 0;

}
