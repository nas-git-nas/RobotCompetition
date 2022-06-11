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
	cv::Point second_point;
	cv::Point third_point;
#ifdef DEBUG_FAKE_MAP
	first_point.x = 500; // 550; //pose.position.x + 200;
	first_point.y = 200; //200; //pose.position.y;
#else
	first_point.x = pose.position.x + 150;
	first_point.y = pose.position.y;
	second_point.x = pose.position.x + 350;
	second_point.y = pose.position.y;
	third_point.x = pose.position.x;
	third_point.y = pose.position.y;
#endif

	std::vector<cv::Point> first_round;
	first_round.push_back(first_point);
	first_round.push_back(second_point);
	//first_round.push_back(third_point);
	
	sps.push_back(first_round);
	
	dm_state = DM_STATE_EXPLORE;
	start_time = time;
	start_position = pose.position;
	nb_collected_bottles = 0;
	nb_collected_fails = 0;
}


void DecisionMaker::stateMachine(Pose pose, Map &map, BottleDetection &bd, Command &command)
{
	// verifyTime
	watchdog();

	// reset command
	resetCommand(command);

	switch(dm_state) {
		case DM_STATE_IDL: {
			ROS_WARN("DM::stateMachine: idl idl idl idl idl idl idl idl idl idl idl idl idl idl idl");
			ros::Duration(10, 0).sleep();
		} break;
			
		case DM_STATE_EXPLORE: {
			explore(pose, map, bd, command);
		} break;
		
		case DM_STATE_MOVE: {
			move(pose, map, command);
		} break;
			
		case DM_STATE_APPROACH: {
			approach(pose, map, bd, command);
		} break;
			
		case DM_STATE_PICKUP_SEND: {
			pickupSend(command);
			ROS_INFO("dm::stateMachine: pickup state send");
			dm_state = DM_STATE_PICKUP_WAIT;			
		} break;
		
		case DM_STATE_PICKUP_WAIT: {
			ROS_INFO("dm::stateMachine: pickup state wait");
		} break;
		
		case DM_STATE_PICKUP_VERIFY: {
			pickupVerify(bd, command);
			ROS_INFO("dm::stateMachine: pickup state verify");
		} break;
			
		case DM_STATE_RETURN: {
			dm_state = DM_STATE_IDL;
			//stateReturn(pose, map, command);
		} break;
			
		case DM_STATE_EMPTY_SEND: {
			dm_state = DM_STATE_IDL;
			//empty(command);
		} break;
		
		case DM_STATE_EMPTY_WAIT: {
			dm_state = DM_STATE_IDL;
			//empty(command);
		} break;
			
		default:
			dm_state = DM_STATE_IDL;
	}
}

void DecisionMaker::watchdog(void)
{
	// init. watchdog
	static ros::Time watchdog_time;
	static uint8_t watchdog_state = DM_STATE_IDL;
	
	// update watchdog
	if(unsigned(watchdog_state) != unsigned(dm_state)) {
		watchdog_time = ros::Time::now();
		watchdog_state = dm_state;
	}
	ros::Duration watchdog_duration = ros::Time::now()-watchdog_time;
	if(DM_VERBOSE_WATCHDOG) {
		ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << watchdog_duration);
	}

	// verify watchdog (if state machine is stuck in a state)	
	switch(dm_state) {
		case DM_STATE_IDL: {

		} break;
			
		case DM_STATE_EXPLORE: {

		} break;
		
		case DM_STATE_MOVE: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_MOVE) {
				dm_state = DM_STATE_EXPLORE;
				if(DM_VERBOSE_WATCHDOG) {
					ROS_WARN("dm::watchdog: move -> explore");
				}
			}
		} break;
			
		case DM_STATE_APPROACH: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_APPROACH) {	
				dm_state = DM_STATE_MOVE;
				if(DM_VERBOSE_WATCHDOG) {
					ROS_WARN("dm::watchdog: approach -> explore");
				}
			}
		} break;
			
		case DM_STATE_PICKUP_WAIT: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_PICKUP) {	
				dm_state = DM_STATE_PICKUP_VERIFY;
				if(DM_VERBOSE_WATCHDOG) {
					ROS_WARN("dm::watchdog: pickup wait -> pickup verify");
				}
			}
		} break;
			
		case DM_STATE_RETURN: {
		
		} break;
			
		case DM_STATE_EMPTY_WAIT: {

		} break;
			
	}	

	// verify if it is time to return
	ros::Duration delta_time = ros::Time::now()-start_time;
	if(delta_time.toSec() > DM_WATCHDOG_END) {
		dm_state = DM_STATE_RETURN;		
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
	Bottle bottle = bd.getBestBottle(map);
	if(bottle.nb_meas >= DM_BOTTLE_NB_MEAS_THR) {
		dm_state = DM_STATE_APPROACH;
		
		if(DM_VERBOSE_EXPLORE) {
			ROS_WARN("DM::explore: explore -> approach");
		}
		return;
	}
	
	// update SP, if true the last set point is reached
	if(updateSP(pose, map)) {
		dm_state = DM_STATE_RETURN;
		return;
	}
	
	// calc. trajectory
	GPP(pose, map, sps[r_idx][sp_idx], command);
}

void DecisionMaker::move(Pose pose, Map &map, Command &command)
{	
	// update SP, if true the last set point is reached
	if(updateSP(pose, map)) {
		dm_state = DM_STATE_RETURN;
		return;
	}
	
	// calc. trajectory
	GPP(pose, map, sps[r_idx][sp_idx], command);
}


void DecisionMaker::approach(Pose pose, Map &map, BottleDetection &bd, Command &command)
{
	// get best bottle
	Bottle bottle = bd.getBestBottle(map);
	
	// verify if a bottle is detected
	if(bottle.position.x==0 && bottle.position.y==0) {
		dm_state = DM_STATE_EXPLORE;
		if(DM_VERBOSE_BD) {
			ROS_INFO("DM::approach: no bottles detected");
			ROS_WARN("dm::approach: approach -> explore");
		}
		return;
	}
	
	// verify if point is reasonable
	if(bottle.position.x<0 || bottle.position.x>MAP_SIZE 
		|| bottle.position.y<0 || bottle.position.y>MAP_SIZE) {
		dm_state = DM_STATE_MOVE;
		nb_collected_fails = 0;
		if(DM_VERBOSE_BD) {
			ROS_INFO("DM::approach: best bottle is not reasonable");
			ROS_WARN("dm::approach: approach -> move");
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
	
		// continue only if bottle is well centered
		if(verifyHeading(bd)) {
			dm_state = DM_STATE_PICKUP_SEND;
			pickup_bottle = bottle;
			
			if(DM_VERBOSE_BD) {
				ROS_INFO("DM::approach: opt. position is reached!");
				ROS_WARN("dm::approach: approach -> pickupSend");
			}
			return;
		}
	}
	
	// verify if optimal position is reachable
	if(map.verifyBottleMapPoint(opt_position, 1, 1)) {
		// TODO: clear recorded bolltes list
		//bd.clearRecordedBottles();
		dm_state = DM_STATE_MOVE;
		nb_collected_fails = 0;
		if(DM_VERBOSE_BD) {
			ROS_WARN("DM::approach: opt. position is not reachable!");
			ROS_WARN("dm::approach: approach -> move");
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
		ROS_INFO_STREAM("dm::approach: state=" << unsigned(command.dm_state));
		ROS_INFO_STREAM("dm::approach: stop motor=" << command.stop_motor);
		ROS_INFO_STREAM("dm::approach: nb_nodes=" << command.nb_nodes);			
		for(int i=0; i<command.trajectory_x.size(); i++) {
			ROS_INFO_STREAM("dm::approach: trajectory[" << i << "] = (" << command.trajectory_x[i] 
									<< "," << command.trajectory_y[i] << ")");
		}
	}
}

bool DecisionMaker::verifyHeading(BottleDetection &bd)
{
	std::array<int,BD_NB_SENSORS> meas = bd.getBottleMeas();
	
	int left_meas = 0;
	int right_meas = 0;
	if(meas[1]>0 && meas[1]<LPP_MEAS_HEADING_THR) {
		left_meas += 1;
	}
	if(meas[2]>0 && meas[2]<LPP_MEAS_HEADING_THR) {
		left_meas += 1;
	}
	if(meas[4]>0 && meas[4]<LPP_MEAS_HEADING_THR) {
		right_meas += 1;
	}
	if(meas[5]>0 && meas[5]<LPP_MEAS_HEADING_THR) {
		right_meas += 1;
	}
	
	// bottle is centered
	if(left_meas==right_meas && meas[3]!=0) {
		return true;
	}
	
	// bottle is not yet centered
	return false;

}

void DecisionMaker::pickupSend(Command &command)
{
	// set command
	command.arm_angle = 1;
	command.air_pump = 1;

	if(DM_VERBOSE_PICKUP) {
		ROS_WARN("dm::pickupSend: pickupSend -> pickupWait");
	}
}

void DecisionMaker::pickupVerify(BottleDetection &bd, Command &command)
{
	// get current ultrasonic measurement
	std::array<int,BD_NB_SENSORS> meas = bd.getBottleMeas();
	
	// count nb. measurments that are not zero
	int nb_meas = 0;
	for(int i=1; i<6; i++) { // ignore sensors on the sides
		if(meas[i] == 0) {
			nb_meas += 1;
		}
	}
	
	//TODO: change condition  and add delai

	// verify if bottle is still detected
	if(nb_meas <= DM_PICKUP_NB_MEAS_THR) {
		dm_state = DM_STATE_EXPLORE;
		nb_collected_bottles += 1;
		nb_collected_fails = 0;
		
		if(DM_VERBOSE_PICKUP) {
			ROS_INFO_STREAM("dm::pickupVerify::nb_meas = " << nb_meas << " -> success");
			ROS_WARN("dm::pickupVerify: pickupVerify -> explore");
		}
	} else {
		dm_state = DM_STATE_APPROACH;
		nb_collected_fails += 1;
		
		if(DM_VERBOSE_PICKUP) {
			ROS_INFO_STREAM("dm::pickupVerify::nb_meas = " << nb_meas << " -> fail -> approach");
			ROS_WARN("dm::pickupVerify: pickupVerify -> approach");
		}
	}
	
	// move on if nb. of fails is too high
	if(nb_collected_fails >= DM_PICKUP_MAX_NB_FAILS) {
		if(DM_VERBOSE_PICKUP) {
			ROS_INFO_STREAM("dm::pickupVerify::nb_fails = " << nb_collected_fails 
									<< " -> move on");
			ROS_WARN("dm::pickupVerify: pickupVerify -> move");
		}
		
		dm_state = DM_STATE_MOVE;
		nb_collected_fails = 0;
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
				dm_state = DM_STATE_EMPTY_SEND;
			}		
		} break;
		
	}
	
	// calc. trajectory and set command
	GPP(pose, map, sp, command);	
}

/*void DecisionMaker::empty(Command &command)
{
	static ros::Time empty_start_time;

	switch(3) {
		case 0: {
			// set timer
			empty_start_time = ros::Time::now();
			
			// change pickup state
			empty_state = 1;
			
			// set command
			command.basket_angle = 1;
		} break;
				
		case 1: {
			// change pickup state if time is elapsed
			ros::Duration elapse_time = ros::Time::now()-empty_start_time;
			if(elapse_time.toSec() > DM_EMPTY_MOVE_DURATION) {

				dm_state = DM_STATE_EXPLORE;
			}
			
			// set command	
			command.basket_angle = 0;
		} break;


	}
}*/

bool DecisionMaker::updateSP(Pose pose, Map &map)
{
	// verify if current set point is reached and update indices if necessary
	if(calcDistance(pose.position, sps[r_idx][sp_idx]) < DM_SP_REACHED_THR) {	
		if(updateSPIndices(pose)) {
			return true;
		}
	}

	
	// verify if current set point is reachable
	while(map.verifyDilatedMapPoint(sps[r_idx][sp_idx], 1, 1)) {
		//TODO: test when SP is in obstacle
		/*int error_x = sps[r_idx][sp_idx].x - pose.position.x;
		int error_y = sps[r_idx][sp_idx].y - pose.position.y;		
		float theta = limitAngle(atan2f(error_y, error_x));
		
		// move set point by DM_SP_CHANGE in direction of robot
		sps[r_idx][sp_idx].x -= cos(theta)*DM_SP_CHANGE;
		sps[r_idx][sp_idx].y -= sin(theta)*DM_SP_CHANGE;*/
		
		// update to next set point
		if(updateSPIndices(pose)) {
			return true;
		}
	}
	return false;
}

bool DecisionMaker::updateSPIndices(Pose pose)
{
	if(sp_idx < sps[r_idx].size()-1) { // there is a next set point in same round
		sp_idx += 1;
	} else if(r_idx < sps.size()-1) { // there is a no set point in same round, but a next round
		sp_idx = 0;
		r_idx += 1;
	} else { // there is no next round => return home
		sp_idx = 0;
		r_idx = 0;
		return true;
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

