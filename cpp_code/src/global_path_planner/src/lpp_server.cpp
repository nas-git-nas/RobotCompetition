#include "ros/ros.h"
#include "global_path_planner/LocalPathPlanner.h"
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "local_path_planner.h"



LocalPathPlanner lpp;
bool lpp_initialized = false;


float rad2degrees(float angle)
{
    return (angle*180.0)/PI;
}

float roundFloat(float number)
{
    return floor(number * 10000.0) / 10000.0;
} 

void initLogLPP(std::vector<cv::Point> nodes, std::vector<int> shortest_path)
{
	
	// create logging instance
	std::ofstream log;
	log.open("log.txt");

	// log set point distance threshold
	log << "--set point size\n";
	log << SET_POINT_DISTANCE_THRESHOLD << "\n";

	// log set points
	log << "--set points\n";
	for(int i=0; i<nodes.size(); i++) {
	  log << nodes[shortest_path[i]].x << "," 
	  	   << roundFloat(nodes[shortest_path[i]].y) << "\n";
	}
	std::cout << "node: (" << nodes[3].x << "," << nodes[3].y << ")" << std::endl;

	// start logging poses
	log << "--poses\n";
	log.close();
}

void logLPP(LocalPathPlanner &lpp)
{
	// create logging instance
	std::ofstream log;
	log.open("log.txt", std::ios_base::app);


	std::array<float,3> pose = lpp.getPose();
	std::vector<cv::Point> set_points = lpp.getSetPoints();
	
	
	log << roundFloat(pose[0]) << "," << roundFloat(pose[1]) << "," 
										<< roundFloat(pose[2]) << "\n";
										
	/*ROS_INFO_STREAM("pose: " << pose[0] << "," << pose[1] << "," << pose[2] 
									 << "," << std::endl);*/

	log.close();
}

bool setLPPCallback(global_path_planner::LocalPathPlanner::Request  &req,
         			  global_path_planner::LocalPathPlanner::Response &res)
{
	std::vector<cv::Point> nodes;
	for(int i=0; i<req.nb_nodes; i++) {
		cv::Point point_temp;
		point_temp.x = req.nodes_x[i];
		point_temp.y = req.nodes_y[i];
		nodes.push_back(point_temp);
	}

	std::vector<int> shortest_path;
	for(int i=0; i<req.nb_nodes_in_path; i++) {
		shortest_path.push_back(req.path[i]);
	}
	
	float new_angle = req.angle;

	lpp.setPoseAndSetPoints(nodes, shortest_path, new_angle);

	res.motor_vel[0] = 0.5;
	res.motor_vel[1] = 0.5;
	res.motor_vel[2] = -0.5;
	res.motor_vel[3] = -0.5;
	
	initLogLPP(nodes, shortest_path);
	lpp_initialized = true;
	return true;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "lpp_service");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("lpp_service", setLPPCallback);
	
	ros::Publisher pub_motor_vel = 
				n.advertise<std_msgs::Float32MultiArray>("motor_vel", 10);
				
	ros::Duration(2).sleep();

	int counter = 0;
	while(ros::ok()) {
		ros::Duration(0.1).sleep();
		ros::spinOnce();
		
		ROS_INFO_STREAM("LPP: " << counter << std::endl);
		
		
		std::array<float,4> motor_vel = lpp.getMotorVelocity();
		
		std_msgs::Float32MultiArray msg_rasp2ard;
  		for(int i=0; i<4; i++) {
  			msg_rasp2ard.data.push_back(motor_vel[i]);
  		}
  		ROS_INFO_STREAM("LPP::motor_vel: (" << msg_rasp2ard.data[0] << "," 
  					 			<< msg_rasp2ard.data[1] << "," << msg_rasp2ard.data[2] 
  					 			<< "," << msg_rasp2ard.data[3] << ")" << std::endl);
  		pub_motor_vel.publish(msg_rasp2ard);
		
		
		if(lpp_initialized) {
			logLPP(lpp);
		}
		
		counter++;
	}



	return 0;
}
