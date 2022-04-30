#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <typeinfo>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>

#include "map.h"
#include "visibility_graph.h"
#include "dijkstra.h"


#define VERBOSE 	false



Map map;




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

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
   
   VisibilityGraph visibility_graph;
	Dijkstra dijkstra;
	
	
   
   
  int counter = 0;
  while(true) {
  		while(!map.new_data) {
  			ros::spinOnce();
  		}
  		map.new_data = false;
  		
  		cv::Point current_position(480.0,410.0); // current robot position
		cv::Point destination(550.0,200.0); // destination of robot
  		
  		map.preprocessData(current_position, destination);
  		

  		visibility_graph.calcGraph(map.getNodes(), map.getNodePolygon());
  		
  		dijkstra.calcPath(visibility_graph.getGraph());
  		
  		//map.draw_graph(visibility_graph.getGraph(), dijkstra.getShortestPath());
  		
  		
  		std::cout<<" spinOnce: " << counter << "\n";
  		counter++;
  		ros::Duration(1, 0).sleep();
	}
  return 0;
}
