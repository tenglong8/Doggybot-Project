#include <doggybot_gazebo/tf_broadcast.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

namespace doggy_tf{
    broadcaster::broadcaster(const std::string& name):
    Node( name ){
    
    subscriber = create_subscription<geometry_msgs::msg::PoseArray>("/pose/info", 100, std::bind(&broadcaster::callback, this, std::placeholders::_1));
    broadcast_pose = std::make_shared<tf2_ros::TransformBroadcaster>(this); 
    
    }
    
    void broadcaster::callback(const geometry_msgs::msg::PoseArray&  msg){
      
      geometry_msgs::msg::TransformStamped Rt;
      Rt.header.stamp = this->get_clock()->now();
      std::string parent, target;
      /*
      if(!this->has_parameter("parent")){
          declare_parameter("parent","");
        }
      if(!this->has_parameter("target")){
      declare_parameter("target","");
      }
      
    get_parameter("parent", parent);
    get_parameter("target", target);
    */
    Rt.header.frame_id = "world";
    Rt.child_frame_id = "body_link";
    
    //tf2::Quaternion q(0.0, 0.0, sin(pose.theta)/2.0, cos(pose.theta)/2.0);
    //Rt.transform.rotation = tf2::toMsg(q);
     for(int i=0; i<msg.poses.size(); i++){
      if( msg.poses[i].position.z>0.6 
       && msg.poses[i].position.z<0.8){
       double qx, qy, qz, qw;
       qx =  msg.poses[i].orientation.x;
       qy =  msg.poses[i].orientation.y;
       qz =  msg.poses[i].orientation.z;
       qw =  msg.poses[i].orientation.w;
       double siny_cosp = 2*(qw*qz - qx*qy);
       double cosy_cosp = 1 - 2*(qy*qy + qz*qz);
       double yaw   = std::atan2(siny_cosp, cosy_cosp);
       
       Rt.transform.translation.x = msg.poses[i].position.x;
       Rt.transform.translation.y = msg.poses[i].position.y;
       Rt.transform.translation.z = msg.poses[i].position.z;
       
       Rt.transform.rotation.x = msg.poses[i].orientation.x;
       Rt.transform.rotation.y = msg.poses[i].orientation.y;
       Rt.transform.rotation.z = msg.poses[i].orientation.z;
       Rt.transform.rotation.w = msg.poses[i].orientation.w;
       }
    }
    
    //Rt.transform.translation.x = 1;
    //Rt.transform.translation.y = 2;
    //Rt.transform.translation.z = 0.0;
    
    broadcast_pose->sendTransform(Rt);
    Rt.header.frame_id = "world";
    Rt.child_frame_id = "odom";
    broadcast_pose->sendTransform(Rt);
    
    }
   
  
}

int main(int argc, char** argv){
   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<doggy_tf::broadcaster>("broadcast"));
    rclcpp::shutdown();  

}
