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

void DecisionMaker::init(Pose pose, ros::Time time, int time_offset)
{

	r_idx = 0;
	sp_idx = 0;
	
	int x = 175;
	int y = 175;
	cv::Point p11(x+200,y), p12(x+350,y),p13(x+400,y-25),p14(x+500,y-25),p15(x+625,y+100), 
		  p16(x+500,y+150),p17(x+400,y+75), p18(x+300,y+100), p19(x+300,y+250),
		  p110(x+200,y+250), p111(x+100,y+100);
	cv::Point p21(x,y+200), p22(x,y+300), p23(x+100,y+300), p24(x+100,y+100);
#ifdef DEBUG_FAKE_MAP
	p11.x = 500; // 550; //pose.position.x + 200;
	p11.y = 200; //200; //pose.position.y;
#endif

	std::vector<cv::Point> first_round, second_round;
	first_round.push_back(p11);
	/*first_round.push_back(p12);
	first_round.push_back(p13);
	first_round.push_back(p14);
	first_round.push_back(p15);
	first_round.push_back(p16);
	first_round.push_back(p17);
	first_round.push_back(p18);
	first_round.push_back(p19);
	first_round.push_back(p110);
	first_round.push_back(p111);
	
	second_round.push_back(p21);
	second_round.push_back(p22);
	second_round.push_back(p23);
	second_round.push_back(p24);*/
	sps.push_back(first_round);
	//sps.push_back(second_round);

	dm_state = DM_STATE_EXPLORE;
	start_time = time;
	starting_time_offset = ros::Duration(time_offset);
	start_position = pose.position;
	nb_collected_bottles = 0;
	nb_approach_fails = 0;
	nb_pickup_fails = 0;
}


void DecisionMaker::stateMachine(Pose pose, Map &map, BottleDetection &bd, Command &command)
{
	if(DM_VERBOSE) {
		ROS_INFO_STREAM("dm::stateMAchine: dm_state = " << dm_state);
	}

	// verifyTime
	watchdog(bd);

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
					
		} break;
		
		case DM_STATE_PICKUP_WAIT: {
		} break;
		
		case DM_STATE_PICKUP_VERIFY: {
			pickupVerify(bd, command);
		} break;
			
		case DM_STATE_RETURN: {
			stateReturn(pose, map, command);
		} break;
		
		case DM_STATE_RECYCLE: {
			recycle(pose, map, command);
		} break;
			
		case DM_STATE_EMPTY_SEND: {
			emptySend(command);
		} break;
		
		case DM_STATE_EMPTY_WAIT: {
		} break;
			
		default:
			dm_state = DM_STATE_IDL;
	}
}

void DecisionMaker::watchdog(BottleDetection &bd)
{
	// init. watchdog
	static ros::Time watchdog_time;
	static uint8_t watchdog_state = DM_STATE_IDL;
	static int nb_pickup_repeats = 0;
	static int nb_empty_repeats = 0;
	static bool competition_end = false;
	
	// update watchdog
	if(unsigned(watchdog_state) != unsigned(dm_state)) {
		watchdog_time = ros::Time::now();
		watchdog_state = dm_state;
	}
	ros::Duration watchdog_duration = ros::Time::now()-watchdog_time;

	// verify watchdog (if state machine is stuck in a state)	
	switch(dm_state) {			
		case DM_STATE_EXPLORE: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_EXPLORE) {
				nb_approach_fails = 0;
				nb_pickup_fails = 0;
			}
		} break;
		
		case DM_STATE_MOVE: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_MOVE) {
				dm_state = DM_STATE_EXPLORE;
				if(DM_VERBOSE_WATCHDOG) {
					ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << watchdog_duration);
					ROS_WARN("dm::watchdog: move -> explore");
				}
			}
		} break;
			
		case DM_STATE_APPROACH: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_APPROACH) {	
				dm_state = DM_STATE_MOVE;
				nb_approach_fails = 0;
				nb_pickup_fails = 0;			
				bd.clearRecordedBottles();
				if(DM_VERBOSE_WATCHDOG) {
					ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << watchdog_duration);
					ROS_WARN("dm::watchdog: approach -> move");
				}
			}
		} break;
			
		case DM_STATE_PICKUP_WAIT: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_PICKUP) {
			
				if(nb_pickup_repeats < DM_PICKUP_NB_REPEATS) {
					dm_state = DM_STATE_PICKUP_SEND;
					nb_pickup_repeats += 1;
					watchdog_time = ros::Time::now();
					if(DM_VERBOSE_WATCHDOG) {
						ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << watchdog_duration);
						ROS_WARN("dm::watchdog: pickup wait -> pickup send");
					}
				} else {						
					dm_state = DM_STATE_PICKUP_VERIFY;
					nb_pickup_repeats = 0;
					
					if(DM_VERBOSE_WATCHDOG) {
						ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << watchdog_duration);
						ROS_WARN("dm::watchdog: pickup wait -> pickup verify");
					}
				}
			}
		} break;

		case DM_STATE_RECYCLE: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_RECYCLE) {
				dm_state = DM_STATE_EMPTY_SEND;

				if(DM_VERBOSE_WATCHDOG) {
					ROS_INFO_STREAM("dm::watchdog_duration: " << watchdog_duration);
					ROS_WARN("dm watchdog: recycle -> empty send");
				}
			}
		} break;
		
		case DM_STATE_EMPTY_WAIT: {
			if(watchdog_duration.toSec() > DM_WATCHDOG_EMPTY) {
				if(nb_empty_repeats < DM_EMPTY_NB_REPEATS) {
					dm_state = DM_STATE_EMPTY_SEND;
					nb_empty_repeats += 1;
					watchdog_time = ros::Time::now();

					if(DM_VERBOSE_WATCHDOG) {
						ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << watchdog_duration);
						ROS_WARN("dm::watchdog: emptyWait -> emptySend");
					}
				} else {						
					dm_state = DM_STATE_EXPLORE;
					nb_empty_repeats = 0;
					
					if(DM_VERBOSE_WATCHDOG) {
					ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << watchdog_duration);
					ROS_WARN("dm::watchdog: emptyWait -> explore");
					}
				}	
			}
		} break;			
	}	

	// verify if it is time to return
	if(!competition_end) {
		ros::Duration delta_time = ros::Time::now()-start_time;
		delta_time += starting_time_offset;
		ROS_INFO_STREAM("dif: " << starting_time_offset.toSec() << " ,time: " << delta_time.toSec());
		if(delta_time.toSec() > DM_WATCHDOG_END) {
			competition_end = true;
			dm_state = DM_STATE_RETURN;
			if(DM_VERBOSE_WATCHDOG) {
				ROS_INFO_STREAM("dm::watchdog::watchdog_duration = " << delta_time.toSec());
				ROS_WARN("dm::watchdog: ... -> return");
			}		
		}
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
	command.move_arm = false;
	command.move_basket = false;
	dm_state = dm_state;
}

void DecisionMaker::explore(Pose pose, Map &map, BottleDetection &bd, Command &command)
{
	// verify if bottle was detected
	Bottle bottle = bd.getBestBottle(map);
	if(bottle.nb_meas >= DM_EXPLORE_NB_MEAS_THR) {
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
	// verify if current set point is reached while approaching, if yes then update it
	if(updateSP(pose, map)) {
		dm_state = DM_STATE_RETURN;
		return;
	}

	// get best bottle
	Bottle bottle = bd.getBestBottle(map);
	
	// verify if a bottle is detected
	if(bottle.position.x==0 && bottle.position.y==0) {	
		nb_approach_fails += 1;
		
		if(DM_VERBOSE_APPROACH) {
			ROS_INFO("DM::approach: no bottles detected");
		}
		
		if(nb_approach_fails >= DM_APPROACH_MAX_NB_FAILS) {
			dm_state = DM_STATE_MOVE;
			nb_approach_fails = 0;
			nb_pickup_fails = 0;			
			bd.clearRecordedBottles();
			
			if(DM_VERBOSE_APPROACH) {
				ROS_INFO_STREAM("dm::approach: nb_approach_fails=" << unsigned(nb_approach_fails)
							<< ", nb_pickup_fails=" << unsigned(nb_pickup_fails));
				ROS_WARN("dm::approach: approach -> move");
			}
			
		} else {
			dm_state = DM_STATE_EXPLORE;
			
			if(DM_VERBOSE_APPROACH) {
				ROS_INFO_STREAM("dm::approach: nb_approach_fails=" << unsigned(nb_approach_fails)
							<< ", nb_pickup_fails=" << unsigned(nb_pickup_fails));
				ROS_WARN("dm::approach: approach -> explore");
			}
		}
		return;
	}
	
	// verify if point is reasonable
	if(bottle.position.x<0 || bottle.position.x>MAP_SIZE 
		|| bottle.position.y<0 || bottle.position.y>MAP_SIZE) {
		dm_state = DM_STATE_MOVE;
		nb_approach_fails = 0;
		nb_pickup_fails = 0;
		bd.clearRecordedBottles();
		
		if(DM_VERBOSE_APPROACH) {
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
	opt_position.x = bottle.position.x - cos(theta_error+pose.heading)*LPP_ARM_LENGTH;
	opt_position.y = bottle.position.y - sin(theta_error+pose.heading)*LPP_ARM_LENGTH;
	
	// verify if optimal position is reached
	// TODO: replace LPP_arm_LENGTH a little bit larger
	if(abs(distance-LPP_ARM_LENGTH)<LPP_BOTTLE_DIST_THR && theta_error<LPP_BOTTLE_ANGLE_THR) {
		int last_nb_meas = bottle.nb_meas;
		bd.clearRecordedBottles();
		if(DM_VERBOSE_APPROACH) {
			ROS_INFO("dm::approach: reached optimal distance -> clear recorded bottles");
		}
		// TODO: test conditions

		// continue only if bottle is well centered
		if(verifyHeading(bd)) {
			dm_state = DM_STATE_PICKUP_SEND;
			pickup_bottle = bottle;
			
			if(DM_VERBOSE_APPROACH) {
				ROS_INFO("DM::approach: opt. position is reached!");
				ROS_WARN("dm::approach: approach -> pickupSend");
			}
			return;
		}/* else {
			// verify if bottle measurment is enough certain
			if(last_nb_meas <= 2) {
				dm_state = DM_STATE_APPROACH;
			}
		}*/
	}
	
	// verify if optimal position is reachable
	if(map.verifyBottleMapPoint(opt_position, 1, 1)) {
		dm_state = DM_STATE_MOVE;
		nb_approach_fails = 0;
		nb_pickup_fails = 0;
		bd.clearRecordedBottles();
		
		if(DM_VERBOSE_APPROACH) {
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
}

bool DecisionMaker::verifyHeading(BottleDetection &bd)
{
	std::array<int,BD_NB_SENSORS> meas = bd.getBottleMeas();
	
	// one of the front sensors detects something
	if(meas[2]>0 || meas[3]>0 || meas[4]>0) {
		return true;
	}

	/*int left_meas = 0;
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
	}*/

	// bottle is not yet centered
	return false;

}

void DecisionMaker::pickupSend(Command &command)
{
	// set command
	command.move_arm = true;
	
	dm_state = DM_STATE_PICKUP_WAIT;
	
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
		if(meas[i] != 0) {
			nb_meas += 1;
		}
	}
	
	//TODO: change condition  and add delai

	// verify if bottle is still detected
	if(nb_meas <= DM_PICKUP_NB_MEAS_THR) {
		dm_state = DM_STATE_EXPLORE;
		nb_collected_bottles += 1;
		nb_approach_fails = 0;
		nb_pickup_fails = 0;
		bd.clearRecordedBottles();
				
		if(DM_VERBOSE_PICKUP) {
			ROS_INFO_STREAM("dm::pickupVerify::nb_meas = " << nb_meas << " -> success");
			ROS_WARN("dm::pickupVerify: pickupVerify -> explore");
		}
	} else {
		dm_state = DM_STATE_APPROACH;
		nb_pickup_fails += 1;
		
		if(DM_VERBOSE_PICKUP) {
			ROS_INFO_STREAM("dm::pickupVerify::nb_meas = " << nb_meas << " -> fail -> approach");
			ROS_INFO_STREAM("dm::pickupVerify: nb_approach_fails=" << unsigned(nb_approach_fails)
						 << ", nb_pickup_fails=" << unsigned(nb_pickup_fails));
			ROS_WARN("dm::pickupVerify: pickupVerify -> approach");
		}
	}
	
	// move on if nb. of fails is too high
	if(nb_pickup_fails >= DM_PICKUP_MAX_NB_FAILS) {
		if(DM_VERBOSE_PICKUP) {
			ROS_INFO_STREAM("dm::pickupVerify: nb_approach_fails=" << unsigned(nb_approach_fails)
					 << ", nb_pickup_fails = " << unsigned(nb_pickup_fails));
			ROS_WARN("dm::pickupVerify: pickupVerify -> move");
		}
		
		dm_state = DM_STATE_MOVE;
		nb_approach_fails = 0;
		nb_pickup_fails = 0;
		bd.clearRecordedBottles();
	}

}

void DecisionMaker::stateReturn(Pose pose, Map &map, Command &command)
{
	cv::Point sp;

	// go corner position in recycling area
	sp.x = DM_RETURN_POSITION_X;
	sp.y = DM_RETURN_POSITION_Y;	
	
	// verify if position is reached
	if(calcDistance(sp, pose.position) < DM_SP_REACHED_THR) {
		dm_state = DM_STATE_RECYCLE;
		
		if(DM_VERBOSE_RETURN) {
			ROS_WARN("dm::pickupSend: stateReturn -> recycle");
		}
	}	
	
	// move set point towards center if it is not reachable
	sp = updateReturnSP(sp, map);			
	
	// calc. trajectory and set command
	GPP(pose, map, sp, command);	
}

void DecisionMaker::recycle(Pose pose, Map &map, Command &command)
{
	cv::Point sp;

	// go corner position in recycling area
	sp.x = DM_RECYCLE_POSITION_X;
	sp.y = DM_RECYCLE_POSITION_Y;	
	
	// verify if position is reached
	if(calcDistance(sp, pose.position) < DM_SP_REACHED_THR) {
		dm_state = DM_STATE_EMPTY_SEND;
		if(DM_VERBOSE_RETURN) {
			ROS_WARN("dm::pickupSend: recycle -> emptySend");
		}
	}
	
	// move set point towards center if it is not reachable
	sp = updateReturnSP(sp, map);

	// calc. trajectory and set command
	GPP(pose, map, sp, command);	
}

void DecisionMaker::emptySend(Command &command)
{
	// set command
	command.move_basket = 1;
	
	dm_state = DM_STATE_EMPTY_WAIT;
	
	if(DM_VERBOSE_RETURN) {
		ROS_WARN("dm::pickupSend: emptySend -> emptyWait");
	}
}

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
		
		// update to next set point
		if(updateSPIndices(pose)) {
			return true;
		}
	}
	return false;
}

cv::Point DecisionMaker::updateReturnSP(cv::Point sp, Map &map)
{
	// verify if current set point is reachable, otherwise move it towards center
	while(map.verifyDilatedMapPoint(sp, 1, 1)) {		
		sp.x += 1;
		sp.y += 1;
	}
	
	return sp;
}

bool DecisionMaker::updateSPIndices(Pose pose)
{
	if(sp_idx < sps[r_idx].size()-1) { // there is a next set point in same round
		sp_idx += 1;
	} else if(r_idx < sps.size()-1) { // there is a no set point in same round, but a next round
		sp_idx = 0;
		r_idx += 1;
		dm_state = DM_STATE_RETURN;
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
	if(DM_VERBOSE_GPP) {
		ROS_INFO("dm::GPP: exit");
	}
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


