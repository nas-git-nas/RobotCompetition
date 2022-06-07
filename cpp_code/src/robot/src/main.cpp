#include "ros/ros.h"
#include "robot/GetPoseSRV.h"
#include "robot/SetTrajectorySRV.h"
#include "robot/CommandSRV.h"
#include "robot/WindowsMsg.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <vector>
#include <typeinfo>
#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "main.h"
#include "map.h"
#include "visibility_graph.h"
#include "dijkstra.h"
#include "bd.h"
#include "dm.h"

/*
* ----- CALLBACK FUNCTION DEFINITIONS -----
*/
void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void arduinoCB(const std_msgs::Int16MultiArray::ConstPtr& msg);

/*
* ----- Service FUNCTIONS DEFINITIONS-----
*/
Pose getPoseSRV(ros::ServiceClient &client_get_pose);
void sendCommandSRV(ros::ServiceClient &client_command, Command &command);
void windowsLogSRV(ros::Publisher& windows_pub, Pose pose);


/*
* ----- GLOBAL CLASSES -----
*/
Map map;
BottleDetection bd;
DecisionMaker dm;


/*
* ----- GLOBAL VARIABLES -----
*/
Pose pose;

/*
* ----- MAIN -----
*/
int main(int argc, char **argv)
{
	// init. ROS
	ros::init(argc, argv, "global_path_planner");
	ros::NodeHandle n;
	ros::Rate loop_rate(3);

	// subscribe to topics
	ros::Subscriber sub_map = n.subscribe("map", 100, mapCB);
	ros::Subscriber sub_arduino = n.subscribe("ard2rasp", 100, arduinoCB);
	
	// publisher of topics
	ros::Publisher windows_pub = 
					n.advertise<robot::WindowsMsg>("windows_pub", 10);
				
	// create client
	ros::ServiceClient client_get_pose = 
	 				n.serviceClient<robot::GetPoseSRV>("controller_get_pose_srv");			
	ros::ServiceClient client_command = 
	 		n.serviceClient<robot::CommandSRV>("controller_command_srv");	 
	ros::ServiceClient client_reset_hector = 
	 				n.serviceClient<robot::SetTrajectorySRV>("reset_map");
	
	
	
	// measure start time
	ros::Time start_time = ros::Time::now();

	// wait until all nodes are initialized
	ros::Duration(2, 0).sleep();
	ROS_INFO_STREAM("\n\n---------- main: start ----------\n\n");


	// current pose
	pose = getPoseSRV(client_get_pose);
	dm.init(pose);

	
	// define command and send init. command
	Command command;
	sendCommandSRV(client_command, command);	

	int counter = 0;
	while(ros::ok()) {
		if(MAIN_VERBOSE) {		
  			ROS_INFO_STREAM("main::counter: " << counter << "\n");
  		}
  		
		// get current pose
		pose = getPoseSRV(client_get_pose);

		
		// make one cycle in state machine
  		dm.stateMachine(pose, map, bd, command);
  		
  		// send command to arduino
  		sendCommandSRV(client_command, command);

		// send log to windows
  		windowsLogSRV(windows_pub, pose);
  		
  		// turn off motors after a certain time
  		ros::Duration delta_time = ros::Time::now()-start_time;
  		if(delta_time.toSec() > 40) {
			Command command_stop;
			sendCommandSRV(client_command, command_stop);
			ros::Duration(10, 0).sleep();			
  		}
  		
  		// make one ros cycle
		ros::spinOnce();
      loop_rate.sleep();
		counter++;
	}
	return 0;
}


/*
* ----- CALLBACK FUNCTIONS -----
*/
void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{	
	map.saveRawData(msg);
}

void arduinoCB(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	std::array<int,BD_NB_SENSORS> meas;
	
	for(int i=0; i<BD_NB_SENSORS; i++) {
		meas[i] = msg->data[i];
		
		if(MAIN_VERBOSE_BD) {
			ROS_INFO_STREAM("main::arduinoCB::US[" << i << "]: " << meas[i]);
		}
	}
	
	// calc. threhsolded and dilated map
	/*if(!map.preprocessData()) {
		ROS_ERROR("gpp::map::preprocessData");
	}	*/

	// update recorded measurements in BD if there are new ones
	bd.setUltrasound(meas, map.getMapThresholded(), pose);

	
	/*Bottle bottle = bd.getBestBottle();
	if(MAIN_VERBOSE_BD) {
		ROS_INFO_STREAM("main::arduinoCB: best bottle = (" << bottle.position.x << "," 
								<< bottle.position.y << ";" << bottle.nb_meas << ")");
	}*/
}


/*
* ----- Service FUNCTIONS -----
*/
Pose getPoseSRV(ros::ServiceClient &client_get_pose)
{
	// current pose
	Pose pose;
#ifdef DEBUG_FAKE_MAP
	pose.position.x = 480; //550;
	pose.position.y = 410; //200;
	pose.heading = 0;
#else
	// call service of LPP to get current pose
	robot::GetPoseSRV srv;
	if(client_get_pose.call(srv)) {	
		pose.position.x = srv.response.x;
		pose.position.y = srv.response.y;
		pose.heading = srv.response.heading;
	} else {
		ROS_ERROR("main::mainGetPose: call client_get_pose failed!");
	}
#endif

	return pose;
}

void sendCommandSRV(ros::ServiceClient &client_command, Command &command)	
{
	if(MAIN_VERBOSE_COMMAND) {
		ROS_INFO_STREAM("main::sendCommandSRV: " << command.stop_motor << ", " << command.nb_nodes);
								
		for(int i=0; i<command.trajectory_x.size(); i++) {
			ROS_INFO_STREAM("trajectory[" << i << "] = (" << command.trajectory_x[i] << "," 
									<< command.trajectory_y[i] << ")");
		}
	}

	robot::CommandSRV srv;
	srv.request.dm_state = command.dm_state;
	if(!command.stop_motor) {
		srv.request.trajectory_x = command.trajectory_x;
		srv.request.trajectory_y = command.trajectory_y;
		srv.request.nb_nodes = command.nb_nodes;
	}
	srv.request.stop_motor = command.stop_motor;
	//srv.request.arm_angle = command.arm_angle;
	//srv.request.basket_angle = command.basket_angle;
	//srv.request.air_pump = command.air_pump;

	if(!client_command.call(srv)) {
		ROS_ERROR("main::sendCommandSRV: Failed to call service!");
	}	
}

void windowsLogSRV(ros::Publisher& windows_pub, Pose pose)
{


	std::vector<cv::Point> nodes = map.getNodes();
	std::vector<int> polygons = map.getNodePolygon();
	std::vector<int> path = dm.getShortestPath();
	
	robot::WindowsMsg msg;

	msg.nb_nodes = nodes.size();	
	for(int i=0; i<nodes.size(); i++) {

		msg.polygons.push_back(polygons[i]);
		msg.nodes_x.push_back(nodes[i].x);
		msg.nodes_y.push_back(nodes[i].y);
	}

	msg.nb_path_nodes = path.size();
	for(int i=0; i<path.size(); i++) {
		msg.path.push_back(path[i]);
	}
	
	msg.heading = pose.heading;

	windows_pub.publish(msg);
}



/*
* ----- MAIN FUNCTIONS -----
*/
/*void mainStopMotors(ros::ServiceClient &client_set_trajectory)
	
{
	robot::SetTrajectorySRV srv;
	srv.request.stop_motor = true;

	if(client_set_trajectory.call(srv))
	{
	 ROS_INFO_STREAM("main::mainStopMotors: stop robot");
	}
	else
	{
	 ROS_ERROR("Failed to call service lpp_service");
	}	
}

Pose mainGetPose(ros::ServiceClient &client_get_pose)
{
	// current pose
	Pose pose;
	
#ifdef DEBUG_FAKE_MAP
	pose.position.x = 550;
	pose.position.y = 200;
	pose.heading = 0;
#else
	// call service of LPP to get current pose
	robot::GetPoseSRV srv;
	if(client_get_pose.call(srv)) {	
		pose.position.x = srv.response.x;
		pose.position.y = srv.response.y;
		pose.heading = srv.response.heading;
	} else {
		ROS_ERROR("main::mainGetPose: call client_get_pose failed!");
	}
#endif

	return pose;
}

void mainGPP(ros::ServiceClient &client_set_trajectory, 
					ros::ServiceClient &client_get_pose,
					ros::ServiceClient &client_reset_hector,
					Dijkstra& dijkstra,
					cv::Point destination)
{
	// get current pose
	Pose pose = mainGetPose(client_get_pose);
	
#ifdef DEBUG_FAKE_MAP
	pose.position.x = 480;
	pose.position.y = 410;
	destination.x = 550; //pose.position.x + 200;
	destination.y = 200; //pose.position.y;
#endif

	if(MAIN_VERBOSE_GPP) {
		ROS_INFO_STREAM("dm::gpp::position: (" << pose.position.x 
							 << "," << pose.position.y << ")");
		ROS_INFO_STREAM("dm::gpp::destination: (" << destination.x 
							 << "," << destination.y << ")");
	}

	VisibilityGraph visibility_graph;
	
	if(map.calcPolygons(pose.position, destination)) {
	
		//TODO: test it
		std_srvs::Trigger reset_map_srv;
		if(client_reset_hector.call(reset_map_srv)) {
			if(MAIN_VERBOSE_GPP) {
				ROS_INFO_STREAM("main::GPP: reset hector map");
			}
		} else {
			ROS_ERROR("main::GPP: reset_map_srv failed");
		}
	}
	std::vector<cv::Point> nodes = map.getNodes();

	
  	visibility_graph.calcGraph(nodes, map.getNodePolygon());
  		
  	if(!dijkstra.calcPath(visibility_graph.getGraph())) {
  		ROS_ERROR("dm::gpp::dijkstra: failed to find shortest path");
  		mainStopMotors(client_set_trajectory);
  	} else {

	  	std::vector<int> path = dijkstra.getShortestPath();
	  	
#ifndef DEBUG_WITHOUT_LPP
		std::vector<uint16_t> trajectory_x;
		std::vector<uint16_t> trajectory_y;
		
		for(int i=0; i<path.size(); i++) {
			trajectory_x.push_back(nodes[path[i]].x);
			trajectory_y.push_back(nodes[path[i]].y);
		}
		
		robot::SetTrajectorySRV srv;	
		srv.request.trajectory_x = trajectory_x;
		srv.request.trajectory_y = trajectory_y;
		srv.request.nb_nodes = trajectory_y.size();
		srv.request.stop_motor = 0;
		
		if(client_set_trajectory.call(srv))
		{
			ROS_INFO_STREAM("dm::gpp: update client_set_trajectory" << std::endl);
		}
		else
		{
			ROS_ERROR("Failed to call service lpp_service");
		}
#endif
		
	  	map.draw_graph(visibility_graph.getGraph(), 
	  							dijkstra.getShortestPath());
	}
	//map.printMap();
}*/


/*
* ----- TEST FUNCTIONS -----
*/
/*void testGPP(ros::ServiceClient &client_set_trajectory, 
				 ros::ServiceClient &client_get_pose,
				 ros::ServiceClient &client_reset_hector,
				 Dijkstra& dijkstra)
{
	if(!map.preprocessData()) {
		ROS_ERROR("gpp::map::preprocessData");
	}	

	// get current pose
	Pose pose = mainGetPose(client_get_pose);	
	cv::Point destination;

	static bool destination_set = false;
	if(!destination_set) {
  		destination.x = pose.position.x+20;
  		destination.y = pose.position.y;
  		destination_set = true;
  	}


	mainGPP(client_set_trajectory, client_get_pose, client_reset_hector, 
				dijkstra, destination);
	
}


void testBottleDetection(ros::ServiceClient &client_get_pose)
{
	// get current pose
	Pose pose = mainGetPose(client_get_pose);

#ifdef DEBUG_FAKE_MEAS
	std::array<int,BD_NB_SENSORS> fake_meas = {10,0,0,10,0,0,0};
	bd.setUltrasound(fake_meas);
#endif

	if(MAIN_VERBOSE_BD) {
		ROS_INFO_STREAM("main::testBD::pose (" << pose.position.x << "," 
							 << pose.position.y << "," << pose.heading << ")");
	}

	
	if(!map.preprocessData()) {
		ROS_ERROR("gpp::map::preprocessData");
	}	

	cv::Point best_bottle = bd.calcBestBottle(map.getMapThresholded(), pose);
	
	if(MAIN_VERBOSE_BD) {
		ROS_INFO_STREAM("main::bd: best bottle = " << best_bottle.x << "," 
							 << best_bottle.y << ")");
	}

}*/
