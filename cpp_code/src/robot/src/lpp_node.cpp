#include "ros/ros.h"
#include "global_path_planner/LocalPathPlanner.h"
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "dm.h"
#include "lpp.h"



LPP lpp;


/*float rad2degrees(float angle)
{
    return (angle*180.0)/PI;
}

float roundFloat(float number)
{
    return floor(number * 10000.0) / 10000.0;
} 

void initLogLPP(std::vector<cv::Point> trajectory)
{
	
	// create logging instance
	std::ofstream log;
	log.open("log.txt");

	// log set point distance threshold
	log << "--set point size\n";
	log << SET_POINT_DISTANCE_THRESHOLD << "\n";

	// log set points
	log << "--set points\n";
	for(int i=0; i<trajectory.size(); i++) {
	  log << trajectory[i].x << "," << trajectory[i].y << "\n";
	}

	// start logging poses
	log << "--poses\n";
	log.close();
}

void logLPP(LPP &lpp)
{
	// create logging instance
	std::ofstream log;
	log.open("log.txt", std::ios_base::app);


	std::array<float,3> pose = lpp.getPose();
	std::vector<cv::Point> set_points = lpp.getSetPoints();
	
	
	log << roundFloat(pose[0]) << "," << roundFloat(pose[1]) << "," 
										<< roundFloat(pose[2]) << "\n";
										

	log.close();
}*/

bool setLPPCallback(global_path_planner::LocalPathPlanner::Request  &req,
         			  global_path_planner::LocalPathPlanner::Response &res)
{
	if(req.stop_motor) {
		lpp.stopMotors();
		
		if(LPP_NODE_VERBOSE) {
			ROS_INFO_STREAM("lpp_node::setLPPCB: call lpp.stop_motor");
		}
	} else {
		std::vector<cv::Point> trajectory;
		for(int i=0; i<req.nb_nodes; i++) {
			cv::Point point_temp;
			point_temp.x = req.trajectory_x[i];
			point_temp.y = req.trajectory_y[i];
			trajectory.push_back(point_temp);
		}
		
		float new_heading = req.heading;

		lpp.setPoseAndSetPoints(trajectory, new_heading);
		
		if(LPP_NODE_VERBOSE) {
			ROS_INFO_STREAM("lpp_node::setLPPCB: call lpp.setPoseAndSetPoints");
		}

		res.motor_vel[0] = 0.5;
		res.motor_vel[1] = 0.5;
		res.motor_vel[2] = -0.5;
		res.motor_vel[3] = -0.5;
		
	}
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
		
		if(LPP_NODE_VERBOSE) {
			ROS_INFO_STREAM("LPP_node: " << counter << std::endl);
		}
		
		std::array<float,4> motor_vel = lpp.getMotorVelocity();

		std_msgs::Float32MultiArray msg_rasp2ard;
  		for(int i=0; i<4; i++) {
  			msg_rasp2ard.data.push_back(motor_vel[i]);
  		}
  		pub_motor_vel.publish(msg_rasp2ard);

  		if(LPP_NODE_VERBOSE) {
	  		ROS_INFO_STREAM("LPP::motor_vel: (" << msg_rasp2ard.data[0] << "," 
	  					 			<< msg_rasp2ard.data[1] << "," << msg_rasp2ard.data[2] 
	  					 			<< "," << msg_rasp2ard.data[3] << ")" << std::endl);
  		}	
		
		counter++;
	}



	return 0;
}
