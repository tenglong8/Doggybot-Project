#include<doggybot_controller/PID.hpp>

void PID::PIDcontroller(geometry_msgs::msg::Twist &twist, double Px, double Py, double past_Px, double past_Py double deltaT){
   double Kp, Ki, Kd;
   twist.linear.x = Kp*Px + Kd*(Px - past_Px)/deltaT;
   twist.angular.z = Kp*Py + Kd*(Px - past_Py)/deltaT;

}
