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
#include "bd.h"

void BottleDetection::setUltrasound(std::array<int,BD_NB_SENSORS> meas)
{
	ultrasound_meas = meas;
	updated_meas = ros::Time::now();
}

cv::Point BottleDetection::calcBestBottle(cv::Mat map_bottle, Pose pose)
{
	// calc. position of measured bottles
	std::vector<Bottle> new_bottles = calcNewBottles(map_bottle, pose);
	
	// add new bottles to recorded bottles vector and update it
	addNewBottles(new_bottles);
	
	// if not bottles are recorded return nothing
	if(recorded_bottles.empty()) {
		cv::Point nothing;
		nothing.x = 0;
		nothing.y = 0;
		return nothing;
	}
	
	// find recorded bottle that has the most measurements and return it
	int most_meas_index = 0;
	for(int i=1; i<recorded_bottles.size(); i++) {
		if(recorded_bottles[i].nb_meas > recorded_bottles[i-1].nb_meas) {
			most_meas_index = i;
		}
	}				
	return recorded_bottles[most_meas_index].position;
}

std::vector<Bottle> BottleDetection::calcNewBottles(cv::Mat map_bottle, Pose pose)
{
	std::vector<Bottle> bottles;

	// if measurement is too old then return empty list
	if(verifyMeasAge()) {
		return bottles;
	}
	
	// verify if there is an object and if the object is a bottle or an obstacle
	for(int i=0; i<BD_NB_SENSORS; i++) {
		// treat first sensors with higher priority
		int j = sensor_priority[i];
		
		// verify if ultrasonic sensor did not measure anything
		if(ultrasound_meas[j]==0) {
			if(BD_VERBOSE) {
				ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << j 
									 << "] did not detect anzthing");
			}
			continue;
		}
		
		if(BD_VERBOSE) {
			ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << j 
								 << "], meas = " << ultrasound_meas[j]);
		}
		
		// convert distance measurement to position in global reference frame
		cv::Point object = convertMeasurement(j, pose);
		
		// verify if object is on map (it is an obstacle)
		bool object_on_map = false;
		for(int k=0; k<2*BD_SEARCH_RANGE; k++) {
			for( int l=0; l<2*BD_SEARCH_RANGE; l++) {
				if(map_bottle.at<uint8_t>(object.x-BD_SEARCH_RANGE+k, 
										 		  object.y-BD_SEARCH_RANGE+l) == 0) {
					object_on_map = true;
					break;
				}
			}
			if(object_on_map) { break; }
		}
		
		// add object to bottles vector if it is not on map
		if(!object_on_map) {
			Bottle temp_bottle;
			temp_bottle.position = object;
			temp_bottle.nb_meas = 1;
			bottles.push_back(temp_bottle);
		}
		
		if(BD_VERBOSE) {
			ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << j 
								 << "] is on map = " << object_on_map);
		}	
	}	

	return bottles;
}

void BottleDetection::addNewBottles(std::vector<Bottle> new_bottles)
{
	// do nothing if there are no new bottles
	if(new_bottles.empty()) {
		return;
	}

	// add newly measured bottles to recorded bottles
	for(int i=0; i<new_bottles.size(); i++) {
		bool bottle_recorded = false;
		for(int j=0; j<recorded_bottles.size(); j++) {
			// verify if two bottles are the same
			if(calcDistance(new_bottles[i], recorded_bottles[j]) < BD_BOTTLE_THR) {		
				// set position to mean between new measurement and recorded position
				updateRecordedBottle(new_bottles[i], recorded_bottles[j]);
				bottle_recorded = true;
				break;
			}
		}
		
		// add measurement to recorded bottles if it is not yet registered
		if(!bottle_recorded) {
			recorded_bottles.push_back(new_bottles[i]);
		}	
	}
	
	// merge recorded measurements if they are close
	int nb_bottles = recorded_bottles.size();
	for(int i=0; i<nb_bottles; i++) {
		for(int j=0; j<i; j++) {
			if(calcDistance(recorded_bottles[i], recorded_bottles[j]) < BD_BOTTLE_THR) {	
				// merge two bottles and delete one of them
				updateRecordedBottle(recorded_bottles[i], recorded_bottles[j]);
				recorded_bottles.erase(recorded_bottles.begin() + i);
				i -= 1;
				nb_bottles -= 1;
				break;
			}
		}
	}	
}

bool BottleDetection::verifyMeasAge(void)
{	
	// calc. time since ultrasonic measurements were updated
	ros::Duration delta_time = ros::Time::now() - updated_meas;
	
	// if time is too long return empty vector
	if(delta_time.toSec() > BD_UPDATE_TIME_LIMIT) {
		if(BD_VERBOSE) {
			ROS_INFO_STREAM("bd::verifyAge: updating time too long");
		}
		return true;
	}
	
	return false;
}

cv::Point BottleDetection::convertMeasurement(int sensor, Pose pose)
{
	// position of object in robot frame
	float robot_x = dist_to_robot[sensor][0] 
							+ cos(dist_to_robot[sensor][2])*ultrasound_meas[sensor];
	float robot_y = dist_to_robot[sensor][1] 
							+ sin(dist_to_robot[sensor][2])*ultrasound_meas[sensor];
							
	ROS_INFO_STREAM("robot_x = " << robot_x << ", robot_y = " << robot_y);
	
	// position of object in global frame
	cv::Point object;
	object.x = int(pose.position.x + cos(pose.heading)*robot_x 
											    + sin(pose.heading)*robot_y);
	object.y = int(pose.position.y + sin(pose.heading)*robot_x 
												 + cos(pose.heading)*robot_y);

	return object;
}

int BottleDetection::calcDistance(Bottle b1, Bottle b2)
{
	float delta_x = b1.position.x - b2.position.x;
	float delta_y = b1.position.y - b2.position.y;
	return int( sqrt(delta_x*delta_x + delta_y*delta_y) );
}

void BottleDetection::updateRecordedBottle(Bottle &b1, Bottle &b2)
{
	// calc. weighted position	
	int new_x = int( (b1.position.x*b1.nb_meas + b2.position.x*b2.nb_meas) 
							/ (b1.nb_meas + b2.nb_meas) );
	int new_y = int( (b1.position.y*b1.nb_meas + b2.position.y*b2.nb_meas) 
							/ (b1.nb_meas + b2.nb_meas) );
	
	// update bottles
	b1.position.x = new_x;
	b1.position.y = new_y;
	b1.nb_meas += b2.nb_meas;
	b2.position.x = new_x;
	b2.position.y = new_y;
	b2.nb_meas += b1.nb_meas;	
}
