#include <doggybot_controller/MPC.hpp>

void mpc::MPCcontroller(geometry_msgs::msg::Twist &twist, std::vector<double> state, std::vector<double> target, double deltaT){
   horizon = 100;
   nx = state.size();
   nu = 2;
   
   x = opti.variable(nx, horizon+1);
   u = opti.variable(nu, horizon);
   x_tar = opti.parameter(nx, 1);
   
   x_ref  = opti.parameter(nx, horizon+1);
   
   
   Q = opti.parameter(nx, nx);
   R = opti.parameter(nu, nu);
   
   // cost function
   
   cost = casadi::MX::dot(x - x_ref, casadi::MX::mtimes(Q, x-x_ref )) 
        + casadi::MX::dot(u, casadi::MX::mtimes(R, u));
        
   // optimal target
    
   opti.minimize(cost);
   
       for (int i = 0; i < horizon; i++) {
            opti.subject_to(x(casadi::Slice(), i+1) == x(casadi::Slice(), casadi::Slice(i)) + \
                                mpc::DiffModel(x(casadi::Slice(), casadi::Slice(i)), u(casadi::Slice(), casadi::Slice(i)), deltaT));
        }

   opti.subject_to(x(casadi::Slice(), 0) == x_tar);
   
   solver_options["ipopt.print_level"] = 0;
   solver_options["ipopt.sb"] = "yes";
   solver_options["ipopt.max_iter"] = 2000;
   solver_options["ipopt.tol"] = 1e-8;
   solver_options["print_time"] = 0;
   solver_options["ipopt.acceptable_obj_change_tol"] = 1e-6;
   opti.solver("ipopt", solver_options);
                           
   opti.set_value(Q, casadi::DM::diag(casadi::DM::vertcat({1.0, 5.0, 0.5})));               
   opti.set_value(R, casadi::DM::diag(casadi::DM::vertcat({0.5, 0.25})));
   opti.set_value(x_ref, casadi::DM::repmat(target, 1, horizon+1));
   
   x_init = casadi::DM::repmat({0.0, 0.0, 0.0}, 1, horizon+1);
   u_init = casadi::DM::repmat({0, 0}, 1, horizon);
 
   std::vector<double> cmd = mpc::solve(state);
   
   twist.linear.x = cmd[0];
   twist.angular.z = cmd[1];

    
}

std::vector<double> mpc::solve(const std::vector<double> x0){
  
  opti.set_initial(x, x_init);
  opti.set_initial(u, u_init);
  opti.set_value(x_tar, x0);
  
  casadi::OptiSol solution = opti.solve();
  
  std::vector<double> cmd;
  casadi::Matrix<double> u0 = solution.value(u)(casadi::Slice(), 0);
  cmd = u0.get_elements();

  x_init = casadi::DM::repmat(solution.value(x)(casadi::Slice(), 0), 1, horizon+1);
  u_init = solution.value(u)(casadi::Slice(), casadi::Slice());
  return cmd;
}


casadi::MX mpc::DiffModel(const casadi::MX& x, const casadi::MX& u, const double dt){
   
   casadi::MX xdot(nx, 1);
   xdot(0) = u(0)*cos(x(2));
   xdot(1) = u(0)*sin(x(2));
   xdot(2) = u(1);
   
   return xdot*dt;
}


