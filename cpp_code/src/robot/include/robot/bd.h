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



// class containing map
class BottleDetection
{
	public:
		/*** VARIABLES ***/

		
		/*** FUNCTIONS ***/
		void setUltrasound(std::array<int,7>  measurements); 
		std::vector<cv::Point> calcBottlePosition(cv::Mat map, Pose pose);

							 
	private:
		/*** VARIABLES ***/
		std::array<int,7> ultrasound_meas;
		ros::Time updated_meas = ros::Time::now();;
		
		/*** FUNCTIONS ***/
		std::vector<cv::Point> convertMeasurements(void);
		
}
