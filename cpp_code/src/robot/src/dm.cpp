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
#include "lpp.h"


std::vector<int> DecisionMaker::getShortestPath(void)
{
	return dijkstra.getShortestPath();
}

void DecisionMaker::init(Pose pose, ros::Time time)
{

	r_idx = 0;
	sp_idx = 0;

	cv::Point first_point;
#ifdef DEBUG_FAKE_MAP
	first_point.x = 500; // 550; //pose.position.x + 200;
	first_point.y = 410; //200; //pose.position.y;
#else
	first_point.x = pose.position.x + 200;
	first_point.y = pose.position.y;
#endif

	std::vector<cv::Point> first_round;
	first_round.push_back(first_point);
	
	sps.push_back(first_round);
	
	dm_state = DM_STATE_EXPLORE;
	start_time = time;
	start_position = pose.position;
	nb_collected_bottles = 0;
}


void DecisionMaker::stateMachine(Pose pose, Map &map, BottleDetection &bd, Command &command)
{
	// verifyTime
	verifyTime();

	// reset command
	resetCommand(command);

	//TODO: add watchdogs

	switch(dm_state) {
		case DM_STATE_IDL: {
			ROS_WARN("DM::stateMachine: idl idl idl idl idl idl idl idl idl idl idl idl idl idl idl");
			ros::Duration(10, 0).sleep();
		} break;
			
		case DM_STATE_EXPLORE: {
			explore(pose, map, bd, command);
			//dm_state = DM_STATE_EXPLORE;
		} break;
			
		case DM_STATE_APPROACH: {
			approach(pose, map, bd, command);
		} break;
			
		case DM_STATE_PICKUP: {
			ROS_INFO("dm::stateMachine: pick up bottle state");
			dm_state = DM_STATE_IDL;
			//pickup(bd, command);
		} break;
			
		case DM_STATE_RETURN: {
		dm_state = DM_STATE_IDL;
			//stateReturn(pose, map, command);
		} break;
			
		case DM_STATE_EMPTY: {
		dm_state = DM_STATE_IDL;
			//empty(command);
		} break;
			
		default:
			dm_state = DM_STATE_IDL;
	}
}

void DecisionMaker::verifyTime(void)
{
	ros::Duration delta_time = ros::Time::now()-start_time;
	if(delta_time.toSec() > 120) {
		dm_state = DM_STATE_IDL;		
	}
}

void DecisionMaker::resetCommand(Command &command)
{
	// set all initial commands
	command.dm_state = dm_state;
	command.trajectory_x.clear();
	command.trajectory_y.clear();
	command.nb_nodes = 0;
	command.stop_motor = true;
	command.arm_angle = 0;
	command.basket_angle = 0;
	command.air_pump = false;
	dm_state = dm_state;
}

void DecisionMaker::explore(Pose pose, Map &map, BottleDetection &bd, Command &command)
{
	// verify if bottle was detected
	Bottle bottle = bd.getBestBottle();
	if(bottle.nb_meas >= DM_BOTTLE_NB_MEAS_THR) {
		if(DM_VERBOSE_EXPLORE) {
			ROS_INFO("DM::explore: go to approach state");
		}
		dm_state = DM_STATE_APPROACH;
		return;
	}

	// verify if current set point is reached and update indices if necessary
	if(updateSPIndices(pose)) {
		dm_state = DM_STATE_RETURN;
		return;
	}
	
	// verify if current set point is reachable
	cv::Mat map_dilated = map.getMapDilated();
	while(map_dilated.at<uint8_t>(sps[r_idx][sp_idx].x, sps[r_idx][sp_idx].y) == 0) {
		int error_x = sps[r_idx][sp_idx].x - pose.position.x;
		int error_y = sps[r_idx][sp_idx].y - pose.position.y;		
		float theta = limitAngle(atan2f(error_y, error_x));
		
		// move set point by DM_SP_CHANGE in direction of robot
		sps[r_idx][sp_idx].x -= cos(theta)*DM_SP_CHANGE;
		sps[r_idx][sp_idx].y -= sin(theta)*DM_SP_CHANGE;
		
		// verify if current set point is reached and update indices if necessary
		if(updateSPIndices(pose)) {
			dm_state = DM_STATE_RETURN;
			return;
		}
	}
	
	// calc. trajectory
	GPP(pose, map, sps[r_idx][sp_idx], command);
}

void DecisionMaker::approach(Pose pose, Map &map, BottleDetection &bd, Command &command)
{
	// get best bottle
	Bottle bottle = bd.getBestBottle();
	
	// verify if point is reasonable
	if(bottle.position.x==0 && bottle.position.y==0) {
		//TODO: add more condition and action
		dm_state = DM_STATE_EXPLORE;

		if(DM_VERBOSE_BD) {
			ROS_INFO("DM::approach: no bottles detected");
		}
		return;
	}
	
	// calc. distance and angle between bottle and robot
	int error_x = bottle.position.x - pose.position.x;
	int error_y = bottle.position.y - pose.position.y;		
	float theta_error = limitAngle(atan2f(error_y, error_x) - pose.heading);
	float distance = sqrtf(error_x*error_x + error_y*error_y);
	
	// calc. optimal position of robot to pick up bottle
	cv::Point opt_position;
	opt_position.x = bottle.position.x - cos(theta_error)*LPP_ARM_LENGTH;
	opt_position.y = bottle.position.x - sin(theta_error)*LPP_ARM_LENGTH;
	
	// verify if optimal position is reached
	if(abs(distance-LPP_ARM_LENGTH)<LPP_BOTTLE_DIST_THR && theta_error<LPP_BOTTLE_ANGLE_THR) {
		dm_state = DM_STATE_PICKUP;
		pickup_bottle = bottle;
		
		if(DM_VERBOSE_BD) {
			ROS_INFO("DM::approach: opt. position is reached!");
		}
		return;
	}
	
	// verify if optimal position is reachable
	cv::Mat map_dilated = map.getMapDilated();
	if(map_dilated.at<uint8_t>(opt_position.x, opt_position.y) == 0) {
		bd.clearRecordedBottles();
		dm_state = DM_STATE_EXPLORE;
		
		if(DM_VERBOSE_BD) {
			ROS_WARN("DM::approach: opt. position is not reachable!");
		}
		return;
	}
	
	// set bottle as destination
	command.stop_motor = false;
	command.nb_nodes = 2;
	command.trajectory_x.push_back(pose.position.x);
	command.trajectory_y.push_back(pose.position.y);
	command.trajectory_x.push_back(bottle.position.x);
	command.trajectory_y.push_back(bottle.position.y);	
	
	if(DM_VERBOSE_BD) {
		ROS_INFO_STREAM("dm::approach: stop motor=" << command.stop_motor);
		ROS_INFO_STREAM("dm::approach: nb_nodes=" << command.nb_nodes);
		ROS_INFO_STREAM("dm::approach: state=" << unsigned(command.dm_state));			
		for(int i=0; i<command.trajectory_x.size(); i++) {
			ROS_INFO_STREAM("dm::approach: trajectory[" << i << "] = (" << command.trajectory_x[i] 
									<< "," << command.trajectory_y[i] << ")");
		}
	}
}



void DecisionMaker::pickup(BottleDetection &bd, Command &command)
{
	static uint8_t pickup_state = DM_PICKUP_STATE_START;
	static ros::Time pickup_start_time;

	switch(pickup_state) {
		case DM_PICKUP_STATE_START: {
			// set timer
			pickup_start_time = ros::Time::now();
			
			// change pickup state
			pickup_state = DM_PICKUP_STATE_MOVE;
			
			// set command
			command.arm_angle = 1;
			command.air_pump = 1;
		} break;
				
		case DM_PICKUP_STATE_MOVE: {
			// change pickup state if time is elapsed
			ros::Duration elapse_time = ros::Time::now()-pickup_start_time;
			if(elapse_time.toSec() > 5) {
				pickup_state = DM_PICKUP_STATE_VERIFY;	
			}
			
			// set command	
			command.arm_angle = 0;
			command.air_pump = 0;
		} break;
				
		case DM_PICKUP_STATE_VERIFY: {
			// verify if bottle is still detected
			Bottle current_bottle = bd.getBestBottle();
			if(calcDistance(current_bottle.position, pickup_bottle.position) > BD_BOTTLE_THR) {
				dm_state = DM_STATE_EXPLORE;
				nb_collected_bottles += 1;
			} else {
				dm_state = DM_STATE_APPROACH;
			}
			
			// change pickup state
			pickup_state = DM_PICKUP_STATE_START;		
		} break;

		default:
			pickup_state = DM_PICKUP_STATE_START;
	}
}

void DecisionMaker::stateReturn(Pose pose, Map &map, Command &command)
{
	static uint8_t return_state = DM_RETURN_STATE_GO_BACK;
	cv::Point sp;

	switch(return_state) {
		case DM_RETURN_STATE_GO_BACK: {	
			// go corner position in recycling area
			sp.x = start_position.x - DM_RECYCLING_OFFSET;
			sp.y = start_position.y - DM_RECYCLING_OFFSET;	
			
			// verify if position is reached
			if(calcDistance(sp, pose.position) < SET_POINT_DISTANCE_THRESHOLD) {
				return_state = DM_RETURN_STATE_TURN;
			}				
		} break;
		
		case DM_RETURN_STATE_TURN: {
			// go corner position in recycling area
			sp.x = start_position.x;
			sp.y = start_position.y;	
			
			// verify if position is reached
			if(calcDistance(sp, pose.position) < SET_POINT_DISTANCE_THRESHOLD) {
				return_state = DM_RETURN_STATE_GO_BACK;
				dm_state = DM_STATE_EMPTY;
			}		
		} break;
		
	}
	
	// calc. trajectory and set command
	GPP(pose, map, sp, command);	
}

void DecisionMaker::empty(Command &command)
{
	static uint8_t empty_state = DM_PICKUP_STATE_START;
	static ros::Time empty_start_time;

	switch(empty_state) {
		case DM_EMPTY_STATE_START: {
			// set timer
			empty_start_time = ros::Time::now();
			
			// change pickup state
			empty_state = DM_EMPTY_STATE_MOVE;
			
			// set command
			command.basket_angle = 1;
		} break;
				
		case DM_EMPTY_STATE_MOVE: {
			// change pickup state if time is elapsed
			ros::Duration elapse_time = ros::Time::now()-empty_start_time;
			if(elapse_time.toSec() > DM_EMPTY_MOVE_DURATION) {
				empty_state = DM_EMPTY_STATE_START;
				dm_state = DM_STATE_EXPLORE;
			}
			
			// set command	
			command.basket_angle = 0;
		} break;

		default:
			empty_state = DM_EMPTY_STATE_START;
	}
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

void DecisionMaker::GPP(Pose pose, Map &map, cv::Point destination, Command &command)
{


	if(DM_VERBOSE_GPP) {
		ROS_INFO_STREAM("DecisionMaker::GPP::position: (" << pose.position.x 
							 << "," << pose.position.y << ")");
		ROS_INFO_STREAM("DecisionMaker::GPP::destination: (" << destination.x 
							 << "," << destination.y << ")");
	}
	
	if(map.calcPolygons(pose.position, destination)) {
		ROS_ERROR("DecisionMaker::GPP: too many large obstacles");
		//TODO: reset map
		/*std_srvs::Trigger reset_map_srv;
		if(client_reset_hector.call(reset_map_srv)) {
			if(DM_VERBOSE_GPP) {
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
  		ROS_ERROR("dm::gpp::dijkstra: failed to find path");
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


/*
#define DM_PICKUP_STATE_MOVE_DOWN	0
#define DM_PICKUP_STATE_ENGAGE		1
#define DM_PICKUP_STATE_MOVE_UP		2
#define DM_PICKUP_STATE_VERIFY		3
#define DM_PICKUP_STATE_DISENGAGE	4
#define DM_PICKUP_STATE_MOVE_FRONT	5
	switch(pickup_state) {
		case DM_PICKUP_STATE_MOVE_DOWN:

			break;		
		case DM_PICKUP_STATE_ENGAGE:
			
			break;
		case DM_PICKUP_STATE_MOVE_UP:
			
			break;
		case DM_PICKUP_STATE_VERIFY:
			
			break;
		case DM_PICKUP_STATE_DISENGAGE:
			
			break;
		case DM_PICKUP_STATE_MOVE_FRONT:
			
			break;
		default:
			pickup_state = DM_PICKUP_STATE_MOVE_FRONT
	}
*/

