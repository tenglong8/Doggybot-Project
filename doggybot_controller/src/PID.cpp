#include<doggybot_controller/PID.hpp>

void pid::PIDcontroller(geometry_msgs::msg::Twist &twist, double Px, double Py, double past_Px, double past_Py, double deltaT){
   double Kp_t, Ki_t, Kd_t;
   double Kp_r, Ki_r, Kd_r;
   Kp_t = 0.3;
   Kd_t = 0.001;
   Kp_r = 0.2;
   Kd_r = 0.001;
   twist.linear.x = Kp_t*Px + Kd_t*(Px - past_Px)/deltaT;
   twist.angular.z = Kp_r*Py + Kd_r*(Py - past_Py)/deltaT;

}
