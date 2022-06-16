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
void windowsLogSRV(ros::Publisher& windows_pub, Pose pose, Command command);


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
	ros::Subscriber sub_map = n.subscribe("map", 1, mapCB);
	ros::Subscriber sub_arduino = n.subscribe("ard2rasp", 1, arduinoCB);
	
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
	
	// retrieve ros parameters
	int starting_time_offset;
	if (n.getParam("starting_time_offset", starting_time_offset)) {
		ROS_INFO("main: got param starting_time_offset");
	} else {
		ROS_ERROR("main: failed to get param starting_time_offset");
		starting_time_offset = 0;
	}
	
	
	// measure start time
	ros::Time start_time = ros::Time::now();

	// wait until all nodes are initialized
	ros::Duration(2, 0).sleep();
	ROS_INFO_STREAM("\n\n---------- main: start ----------\n\n");


	// current pose
	pose = getPoseSRV(client_get_pose);
	dm.init(pose, start_time, starting_time_offset);

	
	// define command and send init. command
	Command command;
	sendCommandSRV(client_command, command);	

	int counter = 0;
	while(ros::ok()) {
		if(MAIN_VERBOSE) {
			std::cout << std::endl;		
  			ROS_INFO_STREAM("main::counter: " << counter);
  		}
  		
		// calc. threhsolded and dilated map
		if(!map.preprocessData()) {
			ROS_ERROR("main::preprocessData: failed");
		}	
  		
  		
		// get current pose
		pose = getPoseSRV(client_get_pose);

		// make one cycle in state machine
  		dm.stateMachine(pose, map, bd, command);

  		// send command to arduino
  		sendCommandSRV(client_command, command);

		// send log to windows
  		windowsLogSRV(windows_pub, pose, command);
  		
  		
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
	bd.setUltrasound(meas, map, pose);

	
	
	if(MAIN_VERBOSE_BD) {
		Bottle bottle = bd.getBestBottle(map);
		ROS_INFO_STREAM("main::arduinoCB: best bottle = (" << bottle.position.x << "," 
								<< bottle.position.y << ";" << bottle.nb_meas << ")");
		ROS_INFO_STREAM("main::arduinoCB: pose = (" << pose.position.x << "," 
																	<< pose.position.y << ")");
	}
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
		ROS_INFO_STREAM("main::sendCommandSRV:");
		ROS_INFO_STREAM("main: stop motor=" << command.stop_motor << ", nb_nodes=" << command.nb_nodes
							<< ", state=" << unsigned(command.dm_state));		
		for(int i=0; i<command.trajectory_x.size(); i++) {
			ROS_INFO_STREAM("trajectory[" << i << "] = (" << command.trajectory_x[i] << "," 
									<< command.trajectory_y[i] << ")");
		}
		ROS_INFO_STREAM("main: move_arm = " << command.move_arm << ", move_basket = " 
								<< command.move_basket);
	}

	robot::CommandSRV srv;
	srv.request.dm_state = command.dm_state;
	if(!command.stop_motor) {
		srv.request.trajectory_x = command.trajectory_x;
		srv.request.trajectory_y = command.trajectory_y;
		srv.request.nb_nodes = command.nb_nodes;
	}
	srv.request.stop_motor = command.stop_motor;
	srv.request.move_arm = command.move_arm;
	srv.request.move_basket = command.move_basket;

	if(!client_command.call(srv)) {
		ROS_ERROR("main::sendCommandSRV: Failed to call service!");
	}
	
}

void windowsLogSRV(ros::Publisher& windows_pub, Pose pose, Command command)
{


	std::vector<cv::Point> nodes = map.getNodes();
	std::vector<int> polygons = map.getNodePolygon();
	std::vector<int> path = dm.getShortestPath();
	Bottle bottle = bd.getBestBottle(map);
	
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
	
	msg.bottle_x = bottle.position.x;
	msg.bottle_y = bottle.position.y;
	msg.bottle_nb_meas = bottle.nb_meas;
	
	msg.state = command.dm_state;
	
	msg.heading = pose.heading;

	windows_pub.publish(msg);
}


