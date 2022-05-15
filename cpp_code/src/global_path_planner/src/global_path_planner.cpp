#include "ros/ros.h"
#include "global_path_planner/LocalPathPlanner.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include <vector>
#include <typeinfo>
#include <iostream>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>

#include "map.h"
#include "visibility_graph.h"
#include "dijkstra.h"
//#include "local_path_planner.h"


#define VERBOSE 	false



Map map;

std::array<float,4> motor_vel_arduino = {0,0,0,0};



/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//std::string s(msg->data.begin(), msg->data.end());
	//ROS_INFO("I heard: [%s]", s.c_str());
	
	map.saveRawData(msg);


	/*if(VERBOSE) {
		int j = 0;
		for(int i:map.data) {
			std::cout<<"j : " << j;
			std::cout<<" occupancy = " << i << "\n";
			j++;
		}
	}*/
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	geometry_msgs::Pose pose1 = msg->pose;

	std::cout << "In side pose callback" << std::endl;
	std::cout << pose1.position.x << std::endl;
}

void arduinoCB(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	ROS_INFO_STREAM("GPP::motor_vel (" << msg->data[0] << "," << msg->data[1] 
				 		  << "," << msg->data[2] << "," << msg->data[3] << ")" 
				 		  << std::endl);
	for(int i=0; i<4; i++) {
		motor_vel_arduino[i] = msg->data[i];
	}
}



void initLPPService(ros::ServiceClient &lpp_client, 
							global_path_planner::LocalPathPlanner &lpp_srv)
	
{
	std::vector<uint16_t> nodes_x;
	std::vector<uint16_t> nodes_y;
	std::vector<uint16_t> shortest_path;
	nodes_x.push_back(0);
	nodes_x.push_back(100);
	nodes_x.push_back(0);
	nodes_y.push_back(0);
	nodes_y.push_back(0);
	nodes_y.push_back(100);
	shortest_path.push_back(0);
	shortest_path.push_back(1);
	shortest_path.push_back(2);	
	float angle = 0;

	lpp_srv.request.nodes_x = nodes_x;
	lpp_srv.request.nodes_y = nodes_y;
	lpp_srv.request.nb_nodes = nodes_x.size();
	lpp_srv.request.path = shortest_path;
	lpp_srv.request.nb_nodes_in_path = shortest_path.size();
	lpp_srv.request.angle = angle;

	if(lpp_client.call(lpp_srv))
	{
	 std::cout << "lpp_client initialiyed" << std::endl;
	}
	else
	{
	 ROS_ERROR("Failed to call service lpp_service");
	}	
}



int main(int argc, char **argv)
{

	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line.
	* For programmatic remappings you can use a different version of init() which takes
	* remappings directly, but for most command-line programs, passing argc and argv is
	* the easiest way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "global_path_planner");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	/**
	* The subscribe() call is how you tell ROS that you want to receive messages
	* on a given topic.  This invokes a call to the ROS
	* master node, which keeps a registry of who is publishing and who
	* is subscribing.  Messages are passed to a callback function, here
	* called chatterCallback.  subscribe() returns a Subscriber object that you
	* must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	* object go out of scope, this callback will automatically be unsubscribed from
	* this topic.
	*
	* The second parameter to the subscribe() function is the size of the message
	* queue.  If messages are arriving faster than they are being processed, this
	* is the number of messages that will be buffered up before beginning to throw
	* away the oldest ones.
	*/
	ros::Subscriber sub = n.subscribe("map", 100, mapCallback);
	ros::Subscriber sub2 = n.subscribe("slam_out_pose", 100, poseCallback);
	ros::Subscriber sub_arduino = n.subscribe("ard2rasp", 10, arduinoCB);
	
	
	ros::Publisher pub_motor_vel = 
				n.advertise<std_msgs::Float32MultiArray>("motor_vel", 10);
				
				
	ros::ServiceClient lpp_client = 
				n.serviceClient<global_path_planner::LocalPathPlanner>("lpp_service");
  global_path_planner::LocalPathPlanner lpp_srv;
	

	
	
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/

	VisibilityGraph visibility_graph;
	Dijkstra dijkstra;


	initLPPService(lpp_client, lpp_srv);
	std::array<float,4> motor_vel;

	ros::Duration(3, 0).sleep();
	std::cout << argv[0] << std::endl;
	ROS_INFO_STREAM("--GPP::start\n");

	int counter = 0;
	while(ros::ok()) {
	
  		ros::Duration(1).sleep();
  		ROS_INFO_STREAM("--GPP: " << counter << "\n");
  		counter++;
  		
  		ros::spinOnce();
  		
  		
  		cv::Point current_position(50.0,50.0); // current robot position
		cv::Point destination(60.0,200.0); // destination of robot
  		
  		if(!map.preprocessData(current_position, destination)) {
  			std::cout<<"ERROR in Map::preprocessData" << std::endl;
  		}
  		

  		visibility_graph.calcGraph(map.getNodes(), map.getNodePolygon());
  		
  		if(dijkstra.calcPath(visibility_graph.getGraph())) {
  			map.draw_graph(visibility_graph.getGraph(), dijkstra.getShortestPath());
  			//map.printMap();
  		}*/
  		
  		/*std_msgs::String msg;
  		
  		std::stringstream ss;
  		ss << "hello world: " << counter;
  		msg.data = ss.str();
  		
  		chatter*/
  		
  				

	}
	return 0;
}
