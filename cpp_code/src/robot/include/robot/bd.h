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
#define BD_BOTTLE_THR 5 // in cm, maximal distance at which two measurements are
								// considered to be the same bottle

// structures
struct Bottle {
	cv::Point position;
	int nb_meas = 0;		// number of times the position was measured
};

// class containing map
class BottleDetection
{
	public:
		/*** VARIABLES ***/

		
		/*** FUNCTIONS ***/
		void setUltrasound(std::array<int,BD_NB_SENSORS> meas);
		cv::Point calcBestBottle(cv::Mat map_bottle, Pose pose);

							 
	private:
		/*** VARIABLES ***/
		std::array<int,BD_NB_SENSORS> ultrasound_meas;
		ros::Time updated_meas;
		std::vector<Bottle> recorded_bottles;
		
		// pose of sensor with respect to robot: {x,y,angle}
		float dist_to_robot[BD_NB_SENSORS][3] = {	{0.0,0.0,0.0},
																{23.5,7.8,0.0},
																{23.5, 0,0.0},
																{23.5,-7.8,0.0},
																{0.0,0.0,0.0},
																{0.0,0.0,0.0},
																{0.0,0.0,0.0}	};
		// rank priority of sensors: most important sensor at first index
		int sensor_priority[BD_NB_SENSORS] = {3,2,4,1,5,0,6};
		
		/*** FUNCTIONS ***/
		std::vector<Bottle> calcNewBottles(cv::Mat map_bottle, Pose pose);
		void addNewBottles(std::vector<Bottle> new_bottles);
		cv::Point convertMeasurement(int sensor, Pose pose);
		bool verifyMeasAge(void);
		int calcDistance(Bottle b1, Bottle b2);
		void updateRecordedBottle(Bottle &b1, Bottle &b2);
		
};

#endif // BD_H
