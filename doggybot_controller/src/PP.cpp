#include <doggybot_controller/PP.hpp>
#include <iostream>  

void pp::PPcontroller(geometry_msgs::msg::Twist &twist, double Px, double Py){
        
        if(abs(Py)>=0.7)
        twist.angular.z = 0.2*Py;
        else{
        twist.linear.x = 0.3*Px;
         twist.angular.z = 0.05*Py;
        }
        
}
