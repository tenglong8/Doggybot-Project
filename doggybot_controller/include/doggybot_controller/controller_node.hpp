#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <pcl_conversions/pcl_conversions.h>    
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h> 
#include <chrono>

namespace controller_node{
 class controller : public rclcpp::Node{
  private:
    double past_x, past_y;
    std::string ctrl_mode;
    std::chrono::steady_clock::time_point start_time;
    
    rclcpp::CallbackGroup::SharedPtr group1;
    rclcpp::CallbackGroup::SharedPtr group2;
    rclcpp::CallbackGroup::SharedPtr group3;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ros_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr robot_pose_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ros_publisher;
    
    geometry_msgs::msg::Twist goal_pos;
    geometry_msgs::msg::Twist robot_pos;
    
  public:
    std::string control_mode;
    controller( const std::string& name );
    
    void callback( const sensor_msgs::msg::PointCloud2::SharedPtr msg );
    void goal_pose_callback( const geometry_msgs::msg::PoseStamped::SharedPtr msg );
    void robot_pose_callback( const geometry_msgs::msg::PoseArray::SharedPtr msg );
    
    void controller_mode( geometry_msgs::msg::Twist &twist, double Px, double Py, double past_Px, double past_Py, double deltaT);
  };
}
