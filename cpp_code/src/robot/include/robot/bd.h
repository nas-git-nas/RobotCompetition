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

#ifndef BD_H // include header file only once
#define BD_H

#define BD_UPDATE_TIME_LIMIT 2 // in s, ultrasonic measurement should not be older than this limit
#define BD_SEARCH_RANGE 4
#define BD_NB_SENSORS 7
#define BD_BOTTLE_THR 5 // in cm, maximal distance at which two measurements are considered to be
								// the same bottle

// structures
struct Bottle {
	cv::Point position;
	int nb_meas = 0;		// number of times the position was confirmed
};

// class containing map
class BottleDetection
{
	public:
		/*** VARIABLES ***/

		
		/*** FUNCTIONS ***/
		void setUltrasound(std::array<int,BD_NB_SENSORS> meas);
		std::vector<cv::Point> calcBottlePosition(cv::Mat map, Pose pose);
		cv::Point closestBottle(cv::Mat map_bottle, Pose pose);

							 
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
		cv::Point convertMeasurement(int sensor, Pose pose);
		bool verifyMeasAge(void);
		int calcDistance(cv::Point p, Bottle b);
		void updateRecordedBottle(cv::Point p1, cv::Point p2, int index);
		
};

#endif // BD_H
