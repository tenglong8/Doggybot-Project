#include <doggybot_controller/PP.hpp>
#include <iostream>  

void pp::PPcontroller(geometry_msgs::msg::Twist &twist, double Px, double Py){
 
        if(Py>=0.1 || Py<= -0.1){
        twist.angular.z = 1.5*Py;}
        else{
        twist.linear.x = 0.3*Px;
         twist.angular.z = Py;
        }
        
}
