#include <doggybot_controller/controller.hpp>
void pp::PPcontroller(geometry_msgs::msg::Twist &twist, double Px, double Py){
   
   twist.linear.x = 0.3*Px;
   twist.angular.z = 0.05*Py;
   
}
