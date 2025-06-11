#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace doggy_tf{

    class broadcaster : public rclcpp::Node{
    private:
       std::shared_ptr<tf2_ros::TransformBroadcaster> broadcast_pose;
       rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber;
    public:
       broadcaster( const std::string& name );
       void callback( const geometry_msgs::msg::Point& pose);
       
    };

}
