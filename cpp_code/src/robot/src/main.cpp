#include "ros/ros.h"
#include "robot/GetPoseSRV.h"
#include "robot/SetTrajectorySRV.h"
#include "robot/WindowsMsg.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_srvs/SetBool.h"
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




/*
* ----- GLOBAL CLASSES -----
*/
Map map;
BottleDetection bd;


/*
* ----- GLOBAL VARIABLES -----
*/


/*
* ----- CALLBACK FUNCTIONS -----
*/
void mapCB(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{	
	map.saveRawData(msg);
}



void poseCovarianceCB(const 		
						geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	/*ROS_INFO_STREAM("Pose (x,y): (" 
						 << std::floor(msg->pose.pose.position.x) << ",\t" 
						 << std::floor(msg->pose.pose.position.y) << ")");*/
	ROS_INFO_STREAM("Pose variance (x,y,theta): (" 
						 << std::floor(msg->pose.covariance[0]) << ",\t" 
						 << std::floor(msg->pose.covariance[7]) << ",\t" 
						 << std::floor(msg->pose.covariance[35]) << ")");
}

void arduinoCB(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	std::array<int,BD_NB_SENSORS> meas;
	ROS_INFO_STREAM("New UM");
	for(int i=0; i<BD_NB_SENSORS; i++) {
		meas[i] = msg->data[i];
		
		ROS_INFO_STREAM("main::arduinoCB::US[" << i << "]: " << meas[i]);
	}
	meas[0] = 0;
	meas[4] = 0;
	meas[5] = 0;
	meas[6] = 0;
	bd.setUltrasound(meas);
	/*ROS_INFO_STREAM("GPP::motor_vel (" << msg->data[0] << "," 
						<< msg->data[1] << "," << msg->data[2] << "," 
						<< msg->data[3] << ")" << std::endl);*/
}


/*
* ----- MAIN FUNCTIONS -----
*/
void mainStopMotors(ros::ServiceClient &client_set_trajectory, 
					 robot::SetTrajectorySRV &srv_set_trajectory)
	
{
	srv_set_trajectory.request.stop_motor = true;

	if(client_set_trajectory.call(srv_set_trajectory))
	{
	 ROS_INFO_STREAM("main::mainStopMotors: stop robot");
	}
	else
	{
	 ROS_ERROR("Failed to call service lpp_service");
	}	
}

Pose mainGetPose(ros::ServiceClient &client_get_pose, 
					  robot::GetPoseSRV &srv_get_pose)
{
	// get current pose
	Pose pose;

	if(client_get_pose.call(srv_get_pose))
	{	
		pose.position.x = srv_get_pose.response.x;
		pose.position.y = srv_get_pose.response.y;
		pose.heading = srv_get_pose.response.heading;
	}
	else
	{
		ROS_ERROR("main::gpp: call client_get_pose failed!");
	}
	
	return pose;
}

void mainGPP(ros::ServiceClient &client_set_trajectory, 
					robot::SetTrajectorySRV &srv_set_trajectory, 
					ros::ServiceClient &client_get_pose, 
					robot::GetPoseSRV &srv_get_pose,
					Dijkstra& dijkstra,
					cv::Point destination)
{
	// get current pose
	Pose pose = mainGetPose(client_get_pose, srv_get_pose);
	
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
	
	map.calcPolygons(pose.position, destination);
	std::vector<cv::Point> nodes = map.getNodes();

	
  	visibility_graph.calcGraph(nodes, map.getNodePolygon());
  		
  	if(!dijkstra.calcPath(visibility_graph.getGraph())) {
  		ROS_ERROR("dm::gpp::dijkstra: failed to find shortest path");
  		mainStopMotors(client_set_trajectory, srv_set_trajectory);
  	} else {

	  	std::vector<int> path = dijkstra.getShortestPath();
	  	
#ifndef DEBUG_WITHOUT_LPP
		std::vector<uint16_t> trajectory_x;
		std::vector<uint16_t> trajectory_y;
		
		for(int i=0; i<path.size(); i++) {
			trajectory_x.push_back(nodes[path[i]].x);
			trajectory_y.push_back(nodes[path[i]].y);
		}
			
		srv_set_trajectory.request.trajectory_x = trajectory_x;
		srv_set_trajectory.request.trajectory_y = trajectory_y;
		srv_set_trajectory.request.nb_nodes = trajectory_y.size();
		srv_set_trajectory.request.stop_motor = 0;
		
		if(client_set_trajectory.call(srv_set_trajectory))
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
}

void mainBottleDetection(ros::ServiceClient &client_get_pose, 
								 robot::GetPoseSRV &srv_get_pose)
{
	// get current pose
	Pose pose = mainGetPose(client_get_pose, srv_get_pose);
	
	std::vector<cv::Point> bottles = bd.calcBottlePosition(map.getMapThresholded(), 
																			 pose);
	
	if(MAIN_VERBOSE_BD) {
		ROS_INFO_STREAM("main::bd: nb. bottles detected = " << bottles.size());
		for(int i=0; i<bottles.size(); i++) {
			ROS_INFO_STREAM("main::bd::bottles[" << i << "] = (" << bottles[i].x << ","
								 << bottles[i].y << ")");
		}
	}

}

void windowsLog(ros::ServiceClient &client_get_pose, 
					 robot::GetPoseSRV &srv_get_pose,
					 ros::Publisher& windows_pub, Dijkstra& dijkstra)
{
	// get current pose
	Pose pose = mainGetPose(client_get_pose, srv_get_pose);

	std::vector<cv::Point> nodes = map.getNodes();
	std::vector<int> polygons = map.getNodePolygon();
	std::vector<int> path = dijkstra.getShortestPath();
	
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
* ----- TEST FUNCTIONS -----
*/
void testGPP(ros::ServiceClient &client_set_trajectory, 
				 robot::SetTrajectorySRV &srv_set_trajectory, 
				 ros::ServiceClient &client_get_pose, 
				 robot::GetPoseSRV &srv_get_pose,
				 Dijkstra& dijkstra)
{
	if(!map.preprocessData()) {
		ROS_ERROR("gpp::map::preprocessData");
	}	

	// get current pose
	Pose pose = mainGetPose(client_get_pose, srv_get_pose);	
	cv::Point destination;

	static bool destination_set = false;
	if(!destination_set) {
  		destination.x = pose.position.x+20;
  		destination.y = pose.position.y;
  		destination_set = true;
  	}


	mainGPP(client_set_trajectory, srv_set_trajectory, 
			  client_get_pose, srv_get_pose, dijkstra, destination);
	
}


void testBottleDetection(ros::ServiceClient &client_get_pose, 
								     robot::GetPoseSRV &srv_get_pose)
{
	// get current pose
	Pose pose = mainGetPose(client_get_pose, srv_get_pose);
	
#ifdef DEBUG_FAKE_MAP
	pose.position.x = 550;
	pose.position.y = 200;
	pose.heading = 0;
#endif

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

	mainBottleDetection(client_get_pose, srv_get_pose);

}


/*
* ----- MAIN -----
*/
int main(int argc, char **argv)
{
	// init. ROS
	ros::init(argc, argv, "global_path_planner");
	ros::NodeHandle n;

	// subscribe to topics
	ros::Subscriber sub = n.subscribe("map", 100, mapCB);
	ros::Subscriber sub3 = n.subscribe("poseupdate", 100, poseCovarianceCB);
	ros::Subscriber sub_arduino = n.subscribe("ard2rasp", 100, arduinoCB);
	
	// publisher of topics
	ros::Publisher windows_pub = 
					n.advertise<robot::WindowsMsg>("windows_pub", 10);
				
	// create client
	ros::ServiceClient client_get_pose = 
	 n.serviceClient<robot::GetPoseSRV>("controller_get_pose_srv");
	robot::GetPoseSRV srv_get_pose;
			
	ros::ServiceClient client_set_trajectory = 
	 n.serviceClient<robot::SetTrajectorySRV>("controller_set_trajectory_srv");
	robot::SetTrajectorySRV srv_set_trajectory;
	
	ros::ServiceClient hector_client = 
	 n.serviceClient<std_srvs::SetBool>("pause_mapping");
	std_srvs::SetBool hector_srv;
	
	Dijkstra dijkstra;

#ifndef DEBUG_WITHOUT_LPP 
	mainStopMotors(client_set_trajectory, srv_set_trajectory);
#endif

	ros::Duration(3, 0).sleep();
	ROS_INFO_STREAM("main: start\n");
	
	
	bool hector_slam = true;
	

	int counter = 0;
	while(ros::ok()) {
		
  		ros::Duration(0.1).sleep();
  		ros::spinOnce();

		if(MAIN_VERBOSE) {		
  			ROS_INFO_STREAM("main::counter: " << counter << "\n");
  		}
  		
  		
  		testBottleDetection(client_get_pose, srv_get_pose);
  		//testGPP(client_set_trajectory, srv_set_trajectory, 
  		//			 client_get_pose, srv_get_posedijkstra);
  		//windowsLog(client_get_pose, srv_get_pose, windows_pub, dijkstra);
  		
  		counter++;			
	

	}
	return 0;
}
