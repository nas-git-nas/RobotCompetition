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

#ifndef BD_H // include header file only once
#define BD_H

#define BD_NB_SENSORS 7
#define BD_UPDATE_TIME_LIMIT 2 // in s, ultrasonic measurement must be younger
#define BD_SEARCH_RANGE 4 // search in this environment for obstacles
								  // [x/y - BD_SEARCH_RANGE, x/y + BD_SEARCH_RANGE]
#define BD_BOTTLE_THR 15 // in cm, maximal distance at which two measurements are
								// considered to be the same bottle
#define BD_NB_MEAS_MAX 30 // maximum number of measurements allowed

// structures
struct Bottle {
	cv::Point position;
	int nb_meas = 0;		// number of times the position was measured
	bool updated = false;
};

// class containing map
class BottleDetection
{
	public:
		/*** VARIABLES ***/

		
		/*** FUNCTIONS ***/
		void setUltrasound(std::array<int,BD_NB_SENSORS> meas, cv::Mat map_bottle, Pose pose);
		Bottle getBestBottle(void);
		void clearRecordedBottles(void);

							 
	private:
		/*** VARIABLES ***/
		std::vector<Bottle> recorded_bottles;
		
		// pose of sensor with respect to robot: {x,y,angle}
		float dist_to_robot[BD_NB_SENSORS][3] = {	{2.0,19.0,1.5708},     // most left one
																{25.0,15.4,0.2618},
																{23.5,7.8,0.0},
																{23.5, 0,0.0},
																{23.5,-7.8,0.0},
																{25.0,-15.4,-0.2618},
																{2.0,-19.0,-1.5708} }; // most right one
		
		/*** FUNCTIONS ***/
		std::vector<Bottle> calcNewBottles(cv::Mat map_bottle, Pose pose, 
														std::array<int,BD_NB_SENSORS> meas);
		void addNewBottles(std::vector<Bottle> new_bottles);
		void ageRecordedBottles(void);
		cv::Point convertMeasurement(int sensor, Pose pose, int measurement);
		bool verifyMeasAge(void);
		int calcDistance(Bottle b1, Bottle b2);
		void updateRecordedBottle(Bottle &b1, Bottle &b2);
		
};

#endif // BD_H
