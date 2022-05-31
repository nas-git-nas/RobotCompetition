#include "ros/ros.h"
#include "robot/LocalPathPlanner.h"
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
#include <tf/tf.h>
#include <vector>
#include <typeinfo>
#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "dm.h"
#include "map.h"
#include "visibility_graph.h"
#include "dijkstra.h"
#include "bd.h"




#define VERBOSE 	false

#define MAP_SIZE_M 5
#define MAP_RESOLUTION 0.01
#define MAP_START_X 0.5
#define MAP_START_Y 0.5
#define MAP_OFFSET_X int(MAP_SIZE_M*MAP_START_X/MAP_RESOLUTION)
#define MAP_OFFSET_Y int(MAP_SIZE_M*MAP_START_Y/MAP_RESOLUTION)
#define MAP_M2PIXEL float(1/MAP_RESOLUTION)
#define DM_CENTER_OFFSET 15


// Global Variables
Map map;
BottleDetection bd;
Pose pose;


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
	
	// subtract offset of LIDAR towards robot center
	pose.position.x -= DM_CENTER_OFFSET; // verify and declare

	// convert quaternions to radians
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,
						  msg->pose.orientation.z, msg->pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);	
	pose.heading = yaw;

	std::cout << "dm::poseCB::pose: (" << pose.position.x << "," << pose.position.y 
				 << "," << pose.heading << ")" << std::endl;
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



void stopMotors(ros::ServiceClient &lpp_client, 
							robot::LocalPathPlanner &lpp_srv)
	
{
	lpp_srv.request.stop_motor = true;

	if(lpp_client.call(lpp_srv))
	{
	 ROS_INFO_STREAM("dm::stopMotors: stop robot");
	}
	else
	{
	 ROS_ERROR("Failed to call service lpp_service");
	}	
}

void gpp(ros::ServiceClient &lpp_client, 
			robot::LocalPathPlanner &lpp_srv, Dijkstra& dijkstra,
			cv::Point destination)
{
	if(DM_VERBOSE_GPP) {
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
  		stopMotors(lpp_client, lpp_srv);
  	} else {

	  	std::vector<int> path = dijkstra.getShortestPath();
	  	
#ifndef DEBUG_WITHOUT_LPP
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
#endif
		
	  	map.draw_graph(visibility_graph.getGraph(), 
	  							dijkstra.getShortestPath());
	}
	//map.printMap();
}

void mainBottleDetection()
{
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

void windowsLog(ros::Publisher& windows_pub, Dijkstra& dijkstra)
{
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

void testGPP(ros::ServiceClient &lpp_client, 
			robot::LocalPathPlanner &lpp_srv, Dijkstra& dijkstra)
{
	if(!map.preprocessData()) {
		ROS_ERROR("gpp::map::preprocessData");
	}	
	
	cv::Point destination;
#ifdef DEBUG_FAKE_MAP
	pose.position.x = 480;
	pose.position.y = 410;
	destination.x = 550; //pose.position.x + 200;
	destination.y = 200; //pose.position.y;
#else
	static bool destination_set = false;
	if(!destination_set) {
  		destination.x = pose.position.x+20;
  		destination.y = pose.position.y;
  		destination_set = true;
  	}
#endif

	gpp(lpp_client, lpp_srv, dijkstra, destination);
	
}


void testMainBottleDetection(void)
{
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

	mainBottleDetection();

}


int main(int argc, char **argv)
{
	// init. ROS
	ros::init(argc, argv, "global_path_planner");
	ros::NodeHandle n;

	// subscribe to topics
	ros::Subscriber sub = n.subscribe("map", 100, mapCallback);
	ros::Subscriber sub2 = n.subscribe("slam_out_pose", 100, poseCallback);
	ros::Subscriber sub3 = n.subscribe("poseupdate", 100, poseCovarianceCB);
	ros::Subscriber sub_arduino = n.subscribe("ard2rasp", 100, arduinoCB);
	
	// publisher of topics
	ros::Publisher windows_pub = 
					n.advertise<robot::WindowsMsg>("windows_pub", 10);
				
	// create client of service			
	ros::ServiceClient lpp_client = 
	 n.serviceClient<robot::LocalPathPlanner>("lpp_service");
	robot::LocalPathPlanner lpp_srv;
	
	ros::ServiceClient hector_client = 
	 n.serviceClient<std_srvs::SetBool>("pause_mapping");
	std_srvs::SetBool hector_srv;
	
	Dijkstra dijkstra;

#ifndef DEBUG_WITHOUT_LPP 
	stopMotors(lpp_client, lpp_srv);
#endif

	ros::Duration(3, 0).sleep();
	ROS_INFO_STREAM("main: start\n");
	
	
	bool hector_slam = true;
	

	int counter = 0;
	while(ros::ok()) {
		
  		ros::Duration(0.1).sleep();
  		ros::spinOnce(); 		
  		//ROS_INFO_STREAM("main::counter: " << counter << "\n");
  		
  		
  		testMainBottleDetection();
  		//testGPP(lpp_client, lpp_srv, dijkstra);
  		//windowsLog(windows_pub, dijkstra);
  		
  		counter++;	
  		
  		
/*  		
  		if(!map.preprocessData()) {
  			ROS_ERROR("gpp::map::preprocessData");
  		}	
  		
  		
#ifdef DEBUG_FAKE_MAP
		pose.position.x = 480 + counter*5;
		pose.position.y = 410;
  		destination.x = 550; //pose.position.x + 200;
  		destination.y = 200; //pose.position.y;
#else
		if(!destination_set) {
	  		destination.x = pose.position.x+20;
	  		destination.y = pose.position.y;
	  		destination_set = true;
	  	}
#endif


		
		float current_heading = pose.heading*180/3.1314;
		//ROS_INFO_STREAM("dm::main::pose (" << pose.position.x << "," << pose.position.y << "," << current_heading << ")");


		//ROS_INFO_STREAM("dm::main call gpp");	
		gpp(lpp_client, lpp_srv, dijkstra, destination);
		windowsLog(windows_pub, dijkstra);

		if(counter%30 == 0) {
			if(hector_slam) {
				hector_slam = false;
			} else {
				hector_slam = true;
			}

			hector_srv.request.data = hector_slam;
			
			if(hector_client.call(hector_srv))
			{
				ROS_INFO_STREAM("----------------dm::gpp: update hector_srv: " << hector_slam);
			}
			else
			{
				ROS_ERROR("Failed to call service lpp_service");
			}
		}
  		
  		//stopMotors(lpp_client, lpp_srv);


#ifndef DEBUG_WITHOUT_LPP  		
  		if(counter>90) {
  			stopMotors(lpp_client, lpp_srv);
  			ros::Duration(5, 0).sleep();
  		}
#endif
*/  		
	

	}
	return 0;
}
