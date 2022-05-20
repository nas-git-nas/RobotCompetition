#include "ros/ros.h"
#include "global_path_planner/LocalPathPlanner.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/tf.h>
#include <vector>
#include <typeinfo>
#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "map.h"
#include "visibility_graph.h"
#include "dijkstra.h"




#define VERBOSE 	false

#define MAP_SIZE_M 5
#define MAP_RESOLUTION 0.01
#define MAP_START_X 0.5
#define MAP_START_Y 0.5
#define MAP_OFFSET_X int(MAP_SIZE_M*MAP_START_X/MAP_RESOLUTION)
#define MAP_OFFSET_Y int(MAP_SIZE_M*MAP_START_Y/MAP_RESOLUTION)
#define MAP_M2PIXEL float(1/MAP_RESOLUTION)



Map map;


struct {
	cv::Point position;
	float heading = 0;
} pose;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{	
	map.saveRawData(msg);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	//geometry_msgs::Pose pose1 = msg->pose;
	
	// convert position in meters to pixels
	pose.position.x = int(msg->pose.position.x*MAP_M2PIXEL + MAP_OFFSET_X);
	pose.position.y = int(msg->pose.position.y*MAP_M2PIXEL + MAP_OFFSET_Y);

	// convert quaternions to radians
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
						  msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);	
	pose.heading = yaw;

	/*std::cout << "pose: (" << pose.position.x << "," << pose.position.y 
				 << "," << pose.heading << ")" << std::endl;*/
}

void arduinoCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	/*ROS_INFO_STREAM("GPP::motor_vel (" << msg->data[0] << "," 
						<< msg->data[1] << "," << msg->data[2] << "," 
						<< msg->data[3] << ")" << std::endl);*/
}



void initLPPService(ros::ServiceClient &lpp_client, 
							global_path_planner::LocalPathPlanner &lpp_srv)
	
{
	std::vector<uint16_t> trajectory_x;
	std::vector<uint16_t> trajectory_y;
	trajectory_x.push_back(0);
	trajectory_x.push_back(0);
	//trajectory_x.push_back(0);
	trajectory_y.push_back(0);
	trajectory_y.push_back(0);
	//trajectory_y.push_back(0);
	float angle = 0;

	lpp_srv.request.trajectory_x = trajectory_x;
	lpp_srv.request.trajectory_y = trajectory_y;
	lpp_srv.request.nb_nodes = trajectory_y.size();
	lpp_srv.request.heading = angle;
	lpp_srv.request.stop_motor = 1;

	if(lpp_client.call(lpp_srv))
	{
	 std::cout << "lpp_client initialized" << std::endl;
	}
	else
	{
	 ROS_ERROR("Failed to call service lpp_service");
	}	
}

void gpp(ros::ServiceClient &lpp_client, 
			global_path_planner::LocalPathPlanner &lpp_srv, 
			cv::Point destination)
{

	/* --- for testing without LIDAR --- */
	//pose.position.x = 480;
	//pose.position.y = 410;
	/* ---                           --- */

	ROS_INFO_STREAM("dm::gpp::position: (" << pose.position.x 
						 << "," << pose.position.y << ")");
	ROS_INFO_STREAM("dm::gpp::destination: (" << destination.x 
						 << "," << destination.y << ")");

	VisibilityGraph visibility_graph;
	Dijkstra dijkstra;
	
	map.calcPolygons(pose.position, destination);
	std::vector<cv::Point> nodes = map.getNodes();
	
  	visibility_graph.calcGraph(nodes, map.getNodePolygon());		
  	if(!dijkstra.calcPath(visibility_graph.getGraph())) {
  		ROS_ERROR("dm::gpp::dijkstra: failed to find shortest path");
  		initLPPService(lpp_client, lpp_srv);
  	} else {

	  	std::vector<int> path = dijkstra.getShortestPath();
	  	
	  	//std::cout << "dm::gpp: path: " << path << std::endl;
	  	
		std::vector<uint16_t> trajectory_x;
		std::vector<uint16_t> trajectory_y;
		
		for(int i=0; i<path.size(); i++) {
			trajectory_x.push_back(nodes[path[i]].x);
			trajectory_y.push_back(nodes[path[i]].y);
		}
			
		lpp_srv.request.trajectory_x = trajectory_x;
		lpp_srv.request.trajectory_y = trajectory_y;
		lpp_srv.request.nb_nodes = trajectory_y.size();
		lpp_srv.request.heading = pose.heading;
		lpp_srv.request.stop_motor = 0;
		
		if(lpp_client.call(lpp_srv))
		{
			ROS_INFO_STREAM("dm::gpp: update lpp_client" << std::endl);
		}
		else
		{
			ROS_ERROR("Failed to call service lpp_service");
		}
		
	  	map.draw_graph(visibility_graph.getGraph(), 
	  							dijkstra.getShortestPath());
	}
	//map.printMap();
	
}



int main(int argc, char **argv)
{
	// init. ROS
	ros::init(argc, argv, "global_path_planner");
	ros::NodeHandle n;

	// subscribe to topics
	ros::Subscriber sub = n.subscribe("map", 100, mapCallback);
	ros::Subscriber sub2 = n.subscribe("slam_out_pose", 100, poseCallback);
	ros::Subscriber sub_arduino = n.subscribe("ard2rasp", 10, arduinoCB);
				
	// create client of service			
	ros::ServiceClient lpp_client = 
	 n.serviceClient<global_path_planner::LocalPathPlanner>("lpp_service");
	global_path_planner::LocalPathPlanner lpp_srv;
	

	initLPPService(lpp_client, lpp_srv);
	ROS_INFO_STREAM("--GPP::start1\n");

	ros::Duration(3, 0).sleep();
	ROS_INFO_STREAM("--GPP::start\n");
	
	bool destination_set = false;
	cv::Point destination;

	int counter = 0;
	while(ros::ok()) {
		
  		ros::Duration(1).sleep();
  		ros::spinOnce(); 		
  		ROS_INFO_STREAM("--GPP: " << counter << "\n");
  		
  		if(!map.preprocessData()) {
  			ROS_ERROR("gpp::map::preprocessData");
  		}		
  		/* --- DECISION MAKER --- */	
  		
  		if(!destination_set) {
	  		destination.x = pose.position.x + 200;
	  		destination.y = pose.position.y;
	  		destination_set = true;
	  	}
  		/* ---                --- */		
  		gpp(lpp_client, lpp_srv, destination);
  		
  		
  		if(counter>20) {
  			initLPPService(lpp_client, lpp_srv);
  			ros::Duration(5, 0).sleep();
  		}
  		
  		counter++;		

	}
	return 0;
}
