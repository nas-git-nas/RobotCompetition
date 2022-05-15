#include "ros/ros.h"
#include "global_path_planner/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<global_path_planner::AddTwoInts>("add_two_ints");
  global_path_planner::AddTwoInts srv;
  
  std::vector<uint16_t> set_points;
  set_points.push_back(1);
  set_points.push_back(2);
  set_points.push_back(3);
  
  srv.request.set_points_x = set_points;
  srv.request.nb_set_points = set_points.size();
  
  if (client.call(srv))
  {
    std::cout << "motor_vel: (" << srv.response.motor_vel[0] << "," 
    			  << srv.response.motor_vel[1] << "," << srv.response.motor_vel[2] 
    			  << "," << srv.response.motor_vel[3] << ")" << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
