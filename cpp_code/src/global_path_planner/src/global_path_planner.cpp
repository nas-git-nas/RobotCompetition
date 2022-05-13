#include "ros/ros.h"
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
#include "local_path_planner.h"


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
	std::cout << "------------------in arduinoCB" << std::endl;
	std::cout << "arduino data: (" << msg->data[0] << "," << msg->data[1] 
				 << "," << msg->data[2] << "," << msg->data[3] << ")" 
				 << std::endl;
	for(int i=0; i<4; i++) {
		motor_vel_arduino[i] = msg->data[i];
	}
	std::cout << "out arduinoCB" << std::endl;
}

float rad2degrees(float angle)
{
    return (angle*180.0)/PI;
}

float roundFloat(float number)
{
    return floor(number * 10000.0) / 10000.0;
} 

void initLPP(LocalPathPlanner &lpp)
{
	cv::Point p1, p2, p3 ,p4;
	p1.x = 0;
	p1.y = 0;
	p2.x = 50;
	p2.y = 0;
	p3.x = 50;
	p3.y = 100;
	p4.x = -100;
	p4.y = 0;

	std::vector<cv::Point> nodes = {p1,p2,p3,p4};
	std::vector<int> shortest_path = {0,1,2,3};
	float new_angle = 0;
	
	lpp.setPoseAndSetPoints(nodes, shortest_path, new_angle);
	
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

	if(VERBOSE_LOCAL_PATH_PLANNER) {
		std::cout << "pose: (" << pose[0] << "," << pose[1] << "," 
					 << rad2degrees(pose[2]) << ") set-point: (" 
				      << set_points[0].x << "," << set_points[0].y 
				      << ")" << std::endl;
	}
		
	log.close();
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
	

	
	
	//ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	/**
	* ros::spin() will enter a loop, pumping callbacks.  With this version, all
	* callbacks will be called from within this thread (the main one).  ros::spin()
	* will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	*/

	VisibilityGraph visibility_graph;
	Dijkstra dijkstra;
	LocalPathPlanner lpp;


	initLPP(lpp);
	std::array<float,4> motor_vel;

	ros::Duration(3, 0).sleep();
	std::cout << argv[0] << std::endl;
	std::cout<<"--start\n";

	int counter = 0;
	while(ros::ok()) {
	
  		ros::Duration(0.1).sleep();
  		std::cout<<"--cycle: " << counter << "\n";
  		counter++;
  		
  		ros::spinOnce();
  		std::cout << "after spinning" << std::endl;
  		
  		std::cout << "recieving: \t(" << motor_vel_arduino[0] << "," 
  					 << motor_vel_arduino[1] << "," << motor_vel_arduino[2] 
  					 << "," << motor_vel_arduino[3] << ")" << std::endl;
  		
  		motor_vel = lpp.getMotorVelocity();
  		logLPP(lpp);
  		std::cout << "vel: \t(" << motor_vel[0]<<"," << motor_vel[1]<<"," 
  					 << motor_vel[2]<<"," << motor_vel[3]<<")" << std::endl;
  		 			
  		std_msgs::Float32MultiArray msg_rasp2ard;
  		for(int i=0; i<4; i++) {
  			msg_rasp2ard.data.push_back(motor_vel[i]);
  		}
  		std::cout << "sending: \t(" << msg_rasp2ard.data[0] << "," 
  					 << msg_rasp2ard.data[1] << "," << msg_rasp2ard.data[2] 
  					 << "," << msg_rasp2ard.data[3] << ")" << std::endl;
  		pub_motor_vel.publish(msg_rasp2ard); 	
  
  		/*while(!map.new_data) {
  			ros::spinOnce();
  		}
  		map.new_data = false;
  		

  		
  		
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
