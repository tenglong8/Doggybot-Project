#include<geometry_msgs/msg/pose.hpp> 


class rrt{
  public:
    vector<geometry::msgs::Pose> RRT();
    
  private:
    typedef std::vector<double> state;
    typedef std::vector<state> path;
    //typedef std::
    //state closest_node()

}
