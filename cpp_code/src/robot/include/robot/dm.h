#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include "nav_msgs/OccupancyGrid.h"
#include <vector>
#include <typeinfo>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "main.h"
#include "bd.h"
#include "map.h"
#include "visibility_graph.h"
#include "dijkstra.h"


#ifndef DM_H // include header file only once
#define DM_H

#define DM_SP_REACHED_THR 10 //cm, should be higher than DM_SP_CHANGE
#define DM_SP_CHANGE 5 //cm, how much set point should be moved towards robot if it is not reachable

#define DM_BOTTLE_NB_MEAS_THR 1
#define DM_RECYCLING_OFFSET			20


#define DM_PICKUP_MAX_NB_FAILS   	3
#define DM_PICKUP_NB_MEAS_THR			1

#define DM_RETURN_STATE_GO_BACK		0
#define DM_RETURN_STATE_TURN			1

#define DM_EMPTY_STATE_START			0	
#define DM_EMPTY_STATE_MOVE			1
#define DM_EMPTY_MOVE_DURATION		5 // in s


#define DM_WATCHDOG_MOVE				10
#define DM_WATCHDOG_APPROACH			30
#define DM_WATCHDOG_PICKUP				7
#define DM_WATCHDOG_END					500 // in s	


// class containing map
class DecisionMaker
{
	public:
		/*** VARIABLES ***/
		
		
		/*** FUNCTIONS ***/
		std::vector<int> getShortestPath(void);
		void init(Pose pose, ros::Time time);
		void stateMachine(Pose pose, Map &map, BottleDetection &bd, Command &command);
		void resetCommand(Command &command);

							 
	private:
		/*** VARIABLES ***/
		uint8_t dm_state = DM_STATE_IDL;
		std::vector<std::vector<cv::Point>> sps; // set points
		uint8_t r_idx = 0; // round index
		uint8_t sp_idx = 0; // round index
		ros::Time start_time;
		cv::Point start_position;
		Bottle pickup_bottle;
		uint8_t nb_collected_bottles = 0;
		uint8_t nb_collected_fails = 0;
		
		/*** CLASSES ***/
		VisibilityGraph visibility_graph;
		Dijkstra dijkstra;

		
		/*** FUNCTIONS ***/
		void watchdog(void);
		void explore(Pose pose, Map &map, BottleDetection &bd, Command &command);
		void move(Pose pose, Map &map, Command &command);
		void approach(Pose pose, Map &map, BottleDetection &bd, Command &command);
		bool verifyHeading(BottleDetection &bd);
		void pickupSend(Command &command);
		void pickupVerify(BottleDetection &bd, Command &command);
		void stateReturn(Pose pose, Map &map, Command &command);
		void empty(Command &command);
		bool updateSP(Pose pose, Map &map);
		bool updateSPIndices(Pose pose);
		void GPP(Pose pose, Map &map, cv::Point destination, Command &command);
		int calcDistance(cv::Point p1, cv::Point p2);
		float limitAngle(float angle);

		
};

#endif // DM_H
