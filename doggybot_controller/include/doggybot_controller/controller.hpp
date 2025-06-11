#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>    
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h> 
namespace rsp{
 class subscriber : public rclcpp::Node{
  private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ros_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros_publisher;
  public:
    subscriber( const std::string& name );
    void callback( const sensor_msgs::msg::PointCloud2::SharedPtr msg );

  };
}
