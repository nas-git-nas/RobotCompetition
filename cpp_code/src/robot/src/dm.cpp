#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <typeinfo>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>


#include "main.h"
#include "dm.h"
#include "bd.h"
#include "map.h"




void DecisionMaker::init(Pose pose)
{

	r_idx = 0;
	sp_idx = 0;

	cv::Point first_point;
	first_point.x = pose.position.x + 100;
	first_point.y = pose.position.y;

	std::vector<cv::Point> first_round;
	first_round.push_back(first_point);
	
	sps.push_back(first_round);

}


void DecisionMaker::stateMachine(Pose pose, Map &map, Command &command, BottleDetection &bd)
{

	// turn everything off
	command.stop_motor = true;
	command.arm_angle = DM_COMMAND_BASKET_REST;
	command.basket_angle = DM_COMMAND_ARM_REST;
	command.air_pump = false;

	switch(dm_state) {
		case DM_STATE_IDL:
			dm_state = DM_STATE_IDL;
			break;		
		case DM_STATE_EXPLORE:
			explore(pose, map, command);
			break;
		case DM_STATE_APPROACH:
			dm_state = DM_STATE_IDL;
			break;
		case DM_STATE_PICKUP:
			dm_state = DM_STATE_IDL;
			break;
		case DM_STATE_RETURN:
			dm_state = DM_STATE_IDL;
			break;
		case DM_STATE_EMPTY:
			dm_state = DM_STATE_IDL;
			break;
		default:
			dm_state = DM_STATE_IDL;
	}
	
	
}

void DecisionMaker::explore(Pose pose, Map &map, Command &command)
{
	// calc. threhsolded and dilated map
	if(!map.preprocessData()) {
		ROS_ERROR("gpp::map::preprocessData");
	}	

	// verify if current set point is reached and update indices if necessary
	if(updateSPIndices(pose)) {
		dm_state = DM_STATE_RETURN;
		return;
	}
	
	// define current set point
	cv::Point sp = sps[r_idx][sp_idx];
	
	// verify if current set point is reachable
	cv::Mat map_dilated = map.getMapDilated();
	while(map_dilated.at<uint8_t>(sps[r_idx][sp_idx].x, sps[r_idx][sp_idx].y) == 0) {
		int error_x = sps[r_idx][sp_idx].x - pose.position.x;
		int error_y = sps[r_idx][sp_idx].y - pose.position.y;		
		float theta = limitAngle(atan2f(error_y, error_x));
		
		// move set point by DM_SP_CHANGE in direction of robot
		sps[r_idx][sp_idx].x -= cos(theta)*DM_SP_CHANGE;
		sps[r_idx][sp_idx].y -= cos(theta)*DM_SP_CHANGE;
		
		// verify if current set point is reached and update indices if necessary
		if(updateSPIndices(pose)) {
			dm_state = DM_STATE_RETURN;
			return;
		}
	}
	
	// calc. trajectory
	GPP(pose, map, sp, command);
}

bool DecisionMaker::updateSPIndices(Pose pose)
{
	// verify if current set point is reached
	if(calcDistance(pose.position, sps[r_idx][sp_idx]) < DM_SP_REACHED_THR) {	
		if(sp_idx < sps[r_idx].size()-1) { // there is a next set point in same round
			sp_idx += 1;
			return false;
		} else if(r_idx < sps.size()-1) { // there is a no set point in same round, but a next round
			sp_idx = 0;
			r_idx += 1;
			return false;
		} else { // there is no next round => return home
			sp_idx = 0;
			r_idx = 0;
			return true;
		}
	}
	return false;
}

void DecisionMaker::GPP(Pose pose, Map &map, cv::Point sp, Command &command)
{

	
#ifdef DEBUG_FAKE_MAP
	pose.position.x = 480;
	pose.position.y = 410;
	sp.x = 550; //pose.position.x + 200;
	sp.y = 200; //pose.position.y;
#endif

	if(MAIN_VERBOSE_GPP) {
		ROS_INFO_STREAM("DecisionMaker::GPP::position: (" << pose.position.x 
							 << "," << pose.position.y << ")");
		ROS_INFO_STREAM("DecisionMaker::GPP::sp: (" << sp.x 
							 << "," << sp.y << ")");
	}

	
	
	if(map.calcPolygons(pose.position, sp)) {
	
		//TODO: test it
		/*std_srvs::Trigger reset_map_srv;
		if(client_reset_hector.call(reset_map_srv)) {
			if(MAIN_VERBOSE_GPP) {
				ROS_INFO_STREAM("main::GPP: reset hector map");
			}
		} else {
			ROS_ERROR("main::GPP: reset_map_srv failed");
		}*/
	}
	std::vector<cv::Point> nodes = map.getNodes();

	
  	visibility_graph.calcGraph(nodes, map.getNodePolygon());
  	
  	std::vector<int> path;
  	if(dijkstra.calcPath(visibility_graph.getGraph())) {
	  	path = dijkstra.getShortestPath();
		
	  	map.draw_graph(visibility_graph.getGraph(), 
	  							dijkstra.getShortestPath());
	  	
	  	// set trajectory commands
	  	command.nb_nodes = path.size();			
		for(int i=0; i<path.size(); i++) {
			command.trajectory_x.push_back(nodes[path[i]].x);
			command.trajectory_y.push_back(nodes[path[i]].y);
		}
		command.stop_motor = false;
  	} else {
  		//TODO: reset map ???
  		ROS_ERROR("dm::gpp::dijkstra: failed to find shortest path");
  		return;

	}
	//map.printMap();

}

int DecisionMaker::calcDistance(cv::Point p1, cv::Point p2)
{
	int delta_x = p1.x - p2.x;
	int delta_y = p1.y - p2.y;
	return int( sqrt(delta_x*delta_x + delta_y*delta_y) );
}

float DecisionMaker::limitAngle(float angle)
{
    while(angle>PI) {
        angle -= 2*PI;
    }
    while(angle<-PI) {
        angle += 2*PI;
    }
    return angle;
}

