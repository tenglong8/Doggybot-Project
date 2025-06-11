#include <doggybot_gazebo/tf_broadcast.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

namespace doggy_tf{
    broadcaster::broadcaster(const std::string& name):
    Node( name ){
    
    subscriber = create_subscription<geometry_msgs::msg::Point>("doggy_pose", 100, std::bind(&broadcaster::callback, this, std::placeholders::_1));
    broadcast_pose = std::make_shared<tf2_ros::TransformBroadcaster>(this); 
    
    }
    
    void broadcaster::callback(const geometry_msgs::msg::Point&  pose){
      
      geometry_msgs::msg::TransformStamped Rt;
      Rt.header.stamp = get_clock()->now();
      std::string parent, target;
      if(!this->has_parameter("parent")){
          declare_parameter("parent","");
        }
      if(!this->has_parameter("target")){
      declare_parameter("target","");
      }
      
    get_parameter("parent", parent);
    get_parameter("target", target);
    Rt.header.frame_id = parent;
    Rt.child_frame_id = target;
    
    //tf2::Quaternion q(0.0, 0.0, sin(pose.theta)/2.0, cos(pose.theta)/2.0);
    //Rt.transform.rotation = tf2::toMsg(q);
    
    Rt.transform.translation.x = pose.x;
    Rt.transform.translation.y = pose.y;
    Rt.transform.translation.z = 0.0;
    
    broadcast_pose->sendTransform(Rt);
    }
   

}
