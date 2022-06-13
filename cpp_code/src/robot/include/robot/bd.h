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
#include "map.h"

#ifndef BD_H // include header file only once
#define BD_H

#define BD_NB_SENSORS 7
#define BD_SEARCH_NEIGHBORHOUD 15 // search in this environment for obstacles								  			// [x/y - BD_SEARCH_RANGE, x/y + BD_SEARCH_RANGE]
#define BD_SEARCH_NB_PIXEL_THR 1 // in neighborhoud there have to be at least this amount of pixels
											// such that it is concidered as occupied
#define BD_BOTTLE_THR 30 // in cm, maximal distance at which two measurements are
								// considered to be the same bottle
#define BD_NB_MEAS_MAX 30 // maximum number of measurements allowed
#define BD_AGE_DEAD_ANGLE_MIN 0.436 // in rad, 25 degrees, defines angle range where the age of the recorded bottles
#define BD_AGE_DEAD_ANGLE_MAX 1.222 // in rad, 70 degrees, is not increased

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
		void setUltrasound(std::array<int,BD_NB_SENSORS> meas, Map map, Pose pose);
		Bottle getBestBottle(Map map);
		std::array<int,BD_NB_SENSORS> getBottleMeas(void);
		void clearRecordedBottles(void);

							 
	private:
		/*** VARIABLES ***/
		std::vector<Bottle> recorded_bottles;
		std::array<int,BD_NB_SENSORS> bottle_meas;
		
		// pose of sensor with respect to robot: {x,y,angle}
		float dist_to_robot[BD_NB_SENSORS][3] = {	{7.0,18.0,0.7854},     // most left one
																{25.0,15.4,0.2618},
																{23.5,7.8,0.0},
																{23.5, 0,0.0},
																{23.5,-7.8,0.0},
																{25.0,-15.4,-0.2618},
																{7.0,-18.0,-0.7854} }; // most right one
		// max. sensor distance, if higher measurement is masked with 0
		int max_meas_distance[BD_NB_SENSORS] = {50, 50, 60, 60, 60, 50, 50};
		
		/*** FUNCTIONS ***/
		std::vector<Bottle> calcNewBottles(Map map, Pose pose,
													  std::array<int,BD_NB_SENSORS> meas);
		void addNewBottles(std::vector<Bottle> new_bottles);
		void ageRecordedBottles(Pose pose);
		cv::Point convertMeasurement(int sensor, Pose pose, int measurement);
		bool verifyMeasAge(void);
		int calcDistance(Bottle b1, Bottle b2);
		void updateRecordedBottle(Bottle &b1, Bottle &b2);
		
};

#endif // BD_H
