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

#define DM_SP_REACHED_THR 17 //cm, should be higher than LPP_SET_POINT_REACHED_THR

#define DM_EXPLORE_NB_MEAS_THR 		1
#define DM_APPROACH_MAX_NB_FAILS   	3
#define DM_PICKUP_MAX_NB_FAILS   	2
#define DM_PICKUP_NB_MEAS_THR			0
#define DM_RECYCLING_OFFSET			25

#define DM_WATCHDOG_EXPLORE			8 // in s, clear nb_pickup_fails and nb_approach_fails 
#define DM_WATCHDOG_MOVE				10 // in s, go to state explore
#define DM_WATCHDOG_APPROACH			30 // in s, go to state move
#define DM_WATCHDOG_PICKUP				5 // in s, length of arm movement when picking up bottles
#define DM_WATCHDOG_RECYCLE				30 //in s, time before emptying basket
#define DM_WATCHDOG_EMPTY 				5 // in s, length of basket movement when emptying it
#define DM_WATCHDOG_END					480 // in s, time before return at the end

#define DM_PICKUP_NB_REPEATS			1
#define DM_EMPTY_NB_REPEATS			1

#define DM_RETURN_POSITION_X			145 // in cm, x position from bottom left corner
#define DM_RETURN_POSITION_Y			145 // in cm, y position from bottom left corner
#define DM_RECYCLE_POSITION_X			170 // in cm
#define DM_RECYCLE_POSITION_Y			170 // in cm


// class containing map
class DecisionMaker
{
	public:
		/*** VARIABLES ***/
		
		
		/*** FUNCTIONS ***/
		std::vector<int> getShortestPath(void);
		void init(Pose pose, ros::Time time, int time_offset);
		void stateMachine(Pose pose, Map &map, BottleDetection &bd, Command &command);
		void resetCommand(Command &command);

							 
	private:
		/*** VARIABLES ***/
		uint8_t dm_state = DM_STATE_IDL;
		std::vector<std::vector<cv::Point>> sps; // set points
		uint8_t r_idx = 0; // round index
		uint8_t sp_idx = 0; // round index
		ros::Time start_time;
		ros::Duration starting_time_offset; // time already passed before starting the node
		cv::Point start_position;
		Bottle pickup_bottle;
		uint8_t nb_collected_bottles = 0;
		uint8_t nb_approach_fails = 0;
		uint8_t nb_pickup_fails = 0;
		
		/*** CLASSES ***/
		VisibilityGraph visibility_graph;
		Dijkstra dijkstra;

		
		/*** FUNCTIONS ***/
		void watchdog(BottleDetection &bd);
		void explore(Pose pose, Map &map, BottleDetection &bd, Command &command);
		void move(Pose pose, Map &map, Command &command);
		void approach(Pose pose, Map &map, BottleDetection &bd, Command &command);
		bool verifyHeading(BottleDetection &bd);
		void pickupSend(Command &command);
		void pickupVerify(BottleDetection &bd, Command &command);
		void stateReturn(Pose pose, Map &map, Command &command);
		void recycle(Pose pose, Map &map, Command &command);
		void emptySend(Command &command);
		bool updateSP(Pose pose, Map &map);
		bool updateSPIndices(Pose pose);
		cv::Point updateReturnSP(cv::Point sp, Map &map);
		void GPP(Pose pose, Map &map, cv::Point destination, Command &command);
		int calcDistance(cv::Point p1, cv::Point p2);
		float limitAngle(float angle);

		
};

#endif // DM_H
