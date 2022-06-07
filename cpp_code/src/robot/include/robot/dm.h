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

#define DM_BOTTLE_NB_MEAS_THR 3
#define DM_BOTTLE_DIST_THR 5 // in cm
#define DM_BOTTLE_ANGLE_THR 0.0873 // rad, equal to 5 degrees




// class containing map
class DecisionMaker
{
	public:
		/*** VARIABLES ***/
		
		
		/*** FUNCTIONS ***/
		std::vector<int> getShortestPath(void);
		void init(Pose pose);
		void stateMachine(Pose pose, Map &map, BottleDetection &bd, Command &command);

							 
	private:
		/*** VARIABLES ***/
		uint8_t dm_state = DM_STATE_IDL;
		std::vector<std::vector<cv::Point>> sps; // set points
		uint8_t r_idx = 0; // round index
		uint8_t sp_idx = 0; // round index
		
		/*** CLASSES ***/
		VisibilityGraph visibility_graph;
		Dijkstra dijkstra;

		
		/*** FUNCTIONS ***/
		void explore(Pose pose, Map &map, BottleDetection &bd, Command &command);
		void approach(Pose pose, Map &map, BottleDetection &bd, Command &command);
		bool updateSPIndices(Pose pose);
		void GPP(Pose pose, Map &map, Command &command);
		int calcDistance(cv::Point p1, cv::Point p2);
		float limitAngle(float angle);

		
};

#endif // DM_H
