#include "ros/ros.h"
#include "global_path_planner/AddTwoInts.h"

bool add(global_path_planner::AddTwoInts::Request  &req,
         global_path_planner::AddTwoInts::Response &res)
{
	//res.sum = req.a + req.b;
	int sum = 0;
	for(int i=0; i<req.nb_set_points; i++) {
		sum += req.set_points_x[i];
	}

	res.motor_vel[0] = float(sum);
	res.motor_vel[1] = 0.5;
	res.motor_vel[2] = -0.5;
	res.motor_vel[3] = -0.5;

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
