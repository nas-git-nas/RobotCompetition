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

void BottleDetection::setUltrasound(std::array<int,BD_NB_SENSORS> meas)
{
	ultrasound_meas = meas;
	updated_meas = ros::Time::now();
}


cv::Point BottleDetection::closestBottle(cv::Mat map_bottle, Pose pose))
{
	std::vector<cv::Point> new_bottles = calcBottlePosition(map_bottle, pose);
	
	for(int i=0; i<new_bottles.size(); i++) {
		bool bottle_recorded = false;
		for(int j=0; j<recorded_bottles.size(); j++) {
			// verify if two bottles are the same
			if(calcDistance(new_bottles[i], recorded_bottles[j])<BD_BOTTLE_THR) {		
				// set position to mean between new measurement and recorded position
				updateRecordedBottle(new_bottles[i], recorded_bottles[j], j);		
				bottle_recorded = true;
				break;
			}
		}
		
		// add measurement to recorded bottles if it is not yet registered
		if(!bottle_recorded) {
			Bottle b_temp;
			b_temp.position = new_bottles[i];
			b_temp.nb_meas = 0;
			recorded_bottles.push_back(b_temp);
		}	
	}
	
	// merge recorded measurements if they are now closer
	for(int i=0; i<recorded_bottles.size(); i++) {
		for(int j=0; j<i; j++) {
			if(calcDistance(recorded_bottles[i], recorded_bottles[j])<BD_BOTTLE_THR) {	
				// update the measurement that has higher confidance and delete the other one
				if(recorded_bottles[i].nb_meas<recorded_bottles[j].nb_meas) {
					updateRecordedBottle(recorded_bottles[i], recorded_bottles[j], j);
					recorded_bottles.erase(recorded_bottles.begin()+i);
				} else {
					updateRecordedBottle(recorded_bottles[j], recorded_bottles[i], i);
					recorded_bottles.erase(recorded_bottles.begin()+j);		
				}
			}
		}
	}			
				
	

}

std::vector<cv::Point> BottleDetection::calcBottlePosition(cv::Mat map_bottle, 
																				Pose pose)
{
	std::vector<cv::Point> bottles;

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
			bottles.push_back(object);
		}
		
		if(BD_VERBOSE) {
			ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << j 
								 << "] is on map = " << object_on_map);
		}	
	}
	

	return bottles;
}

int BottleDetection::calcDistance(cv::Point p, Bottle b)
{
	float delta_x = p.x - b.position.x;
	float delta_y = p.y - b.position.y;
	return int(sqrt(delta_x*delta_x + delta_y*delta_y));
}

void BottleDetection::updateRecordedBottle(cv::Point p1, cv::Point p2, index)
{
	int nb_meas = recorded_bottles[index].nb_meas;
	recorded_bottles[index].position.x = int( (p2.x*nb_meas + p1.x)/(nb_meas+1) );
	recorded_bottles[index].position.y = int( (p2.y*nb_meas + p1.y/(nb_meas+1) );
	recorded_bottles[index].nb_meas += 1;
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
