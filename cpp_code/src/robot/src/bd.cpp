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


#include "dm.h"
#include "bd.h"

void BottleDetection::setUltrasound(std::array<int,7>  measurements)
{
	ultrasound_meas = measurements;
	updated_meas = ros::Time::now();
}


std::vector<cv::Point> BottleDetection::calcBottlePosition(cv::Mat map, Pose pose)
{
	std::vector<cv::Point> bottles;

	// calc. time since ultrasonic measurements were updated
	ros::Duration delta_time = ros::Time::now() - updated_meas;
	
	// if time is too long return empty vector
	if(delta_time.toSec() > BD_UPDATE_TIME_LIMIT) {
		if(BD_VERBOSE) {
			ROS_INFO_STREAM("bd::calcBottlePosition: updating time too long");
		}
		return bottles;
	}
	
	// verify if there is an object and if the object is a bottle or an obstacle
	for(int i=0; i<bottles.size(); i++) {
		// opject position in robot reference frame
		cv::Point object(ultrasound_meas[i].x,ultrasound_meas[i].y)
		
		// ultrasonic sensor did not measure anything
		if(object.x==0 && object.y==0) {
			continue;
		}
		
		// calc. object position in global reference frame
		object.x = pose.position.x + object.x*cos(pose.heading) - object.y*sin(pose.heading);
		object.y = pose.position.y + object.x*sin(pose.heading) + object.y*cos(pose.heading);
		
		// verify if object is on map (it is an obstacle)
		bool object_on_map = false;
		for(int k=0; k<2*BD_SEARCH_RANGE; k++) {
			for( int l=0; l<2*BD_SEARCH_RANGE; l++) {
				if(map.at<uint8_t>(object.x-BD_SEARCH_RANGE+k, object.y-BD_SEARCH_RANGE+l) == 0) {
					object_on_map = true;
					break;
				}
			}
			if(object_on_map) { break; }
		}
		
		// add object to bottles vector if it is not on map
		if(!object_on_map) {
			bottles.push_back(object);
		}		
	}
	
	// TODO: define logical order of bottles vector
	return bottles;
}

std::vector<cv::Point> BottleDetection::convertMeasurements(void)
{
	std::array<cv::Point,7> position;

	// TODO: convert ultrasonic measurements from distance to position in robot reference frame

	return position;
}
