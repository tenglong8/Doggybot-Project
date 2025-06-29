#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>

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
   casadi::MX x_tar;
   casadi::MX x;
   casadi::DM x_init;
   casadi::MX x_ref;
   casadi::MX u;
   casadi::DM u_init;
   casadi::MX cost;
   
   casadi::MX DiffModel(const casadi::MX& x, const casadi::MX& u, const double dt);
   
   
   //vector<int> DiffModel(vector<int> state, vector<int> input);
   //double mpc::cost(Eigen::Matrix<double, 3, 3> Q, Eigen::Matrix<double, 2, 2> R,
    //   Eigen::Matrix<double, 3, 1> state, Eigen::Matrix<double, 2, 1> input );
 };
 
