#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>

#define MAX_LINEAR_VELOCITY     0.5   // m/s
#define MAX_ANGULAR_VELOCITY    0.5   // rad/s
#define MAX_DELTA_VELOCITY    0.5   // m/s
#define MAX_DELTA_OMEGA    M_PI/2.0   // rad/s

 class mpc{
  public:
   void MPCcontroller(geometry_msgs::msg::Twist &twist, std::vector<double> state, std::vector<double> target, double deltaT);
   std::vector<double> solve(const std::vector<double> x0);
  private:
   casadi::Opti opti;
   int horizon;
   int nx;
   int nu;
   
   casadi::Dict solver_options;
   casadi::MX Q;
   casadi::MX R;
   casadi::MX Qf;
   casadi::MX x_tar;
   casadi::MX x;
   casadi::DM x_init;
   casadi::MX x_ref;
   casadi::MX u;
   casadi::DM u_init;
   casadi::MX cost;
   
   casadi::MX DiffModel(const casadi::MX& x, const casadi::MX& u, const double dt);
   
   
 };
 
