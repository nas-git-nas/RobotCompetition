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

void BottleDetection::setUltrasound(std::array<int,BD_NB_SENSORS> meas, cv::Mat map_bottle, 
												Pose pose)
{
	// verify if measurements are false
	for(int i=0; i<BD_NB_SENSORS; i++) {
		if(meas[i]>BD_ULTRASOUND_MAX_DISTANCE || meas[i]<0) {
			meas[i] = 0;
			if(BD_VERBOSE_SET_ULTRASOUND) {
				ROS_WARN("main::arduinoCB: measurement out of range");
			}
		}
	}

	// calc. position of measured bottles
	std::vector<Bottle> new_bottles = calcNewBottles(map_bottle, pose, meas);
	
	// add new bottles to recorded bottles vector and update it
	addNewBottles(new_bottles);
	
	//	subtract one measurement number of all bottles that were not updated this round
	ageRecordedBottles();
	
	if(BD_VERBOSE_SET_ULTRASOUND) {
		ROS_INFO("bd::setUltrasound: ---- measured bottles ----");
		for(int i=0; i<meas.size(); i++) {
			ROS_INFO_STREAM("bd::setUltrasound: m[" << i << "] = (" << meas[i] << ")");
		}		
		ROS_INFO("bd::setUltrasound: ---- recorded bottles ----");
		for(int i=0; i<recorded_bottles.size(); i++) {
			ROS_INFO_STREAM("bd::setUltrasound: b[" << i << "] = (" << recorded_bottles[i].position.x 
									<< "," << recorded_bottles[i].position.y << ";" 
									<< recorded_bottles[i].nb_meas << ")");		
		}
	}
}

Bottle BottleDetection::getBestBottle(void)
{
	// if no bottles are recorded return nothing
	if(recorded_bottles.empty()) {
		Bottle nothing;
		nothing.position.x = 0;
		nothing.position.y = 0;
		nothing.nb_meas = 0;
		return nothing;
	}
	
	// find recorded bottle that has the most measurements and return it
	int most_meas_index = 0;
	for(int i=1; i<recorded_bottles.size(); i++) {
		if(recorded_bottles[i].nb_meas > recorded_bottles[i-1].nb_meas) {
			most_meas_index = i;
		}
	}				
	return recorded_bottles[most_meas_index];
}

void BottleDetection::clearRecordedBottles(void)
{
	recorded_bottles.clear();
}

std::vector<Bottle> BottleDetection::calcNewBottles(cv::Mat map_bottle, Pose pose, 		
																	 std::array<int,BD_NB_SENSORS> meas)
{
	std::vector<Bottle> bottles;
	
	// verify if there exists a map
	if(map_bottle.empty()) {
		if(BD_VERBOSE_CALC) {
			ROS_WARN("bd::calcBottlePosition: map does not exists");
		}
		return bottles;
	}
	
	/*cv::imwrite("map_bd.jpg", map_bottle);
	ROS_INFO_STREAM("type: " << map_bottle.type());	
	std::cout << "--map start" << std::endl << map_bottle << std::endl 
				 << "--map end" << std::endl;
	ROS_INFO_STREAM("particular: " << map_bottle.at<uint8_t>(10,10));
	ROS_INFO_STREAM("particular: " << unsigned(map_bottle.at<uint8_t>(10,10)));*/
	
	// verify if there is an object and if the object is a bottle or an obstacle
	for(int i=0; i<BD_NB_SENSORS; i++) {
		
		// verify if ultrasonic sensor did not measure anything
		if(meas[i]==0) {
			if(BD_VERBOSE_CALC) {
				ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << i 
									 << "] did not detect anzthing");
			}
			continue;
		}
		
		if(BD_VERBOSE_CALC) {
			ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << i 
								 << "], meas = " << meas[i]);
		}
		
		// convert distance measurement to position in global reference frame
		cv::Point object = convertMeasurement(i, pose, meas[i]);
		
		// verify if object is on map (it is an obstacle)
		bool object_on_map = false;
		for(int k=0; k<2*BD_SEARCH_RANGE; k++) {
		
			// outside map
			if(k<0 || k>MAP_SIZE-1) { continue; }
			
			for( int l=0; l<2*BD_SEARCH_RANGE; l++) {
			
				// outside map
				if(l<0 || l>MAP_SIZE-1) { continue; }
				
				if(unsigned(map_bottle.at<uint8_t>(object.x-BD_SEARCH_RANGE+k, 
										 		  			  object.y-BD_SEARCH_RANGE+l)) == 0) {
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
			temp_bottle.updated = true;
			bottles.push_back(temp_bottle);
		}
		
		if(BD_VERBOSE_CALC) {
			ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << i 
								 << "] is on map = " << object_on_map);
		}	
	}	
	return bottles;
}

void BottleDetection::addNewBottles(std::vector<Bottle> new_bottles)
{
	// do nothing if there are no new bottles
	if(new_bottles.empty()) {
		if(BD_VERBOSE_CALC) {
			ROS_INFO_STREAM("bd::addNewBottles: no new bottles -> return");
		}
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
			if(BD_VERBOSE_CALC) {
				ROS_INFO_STREAM("bd::addNewBottles: add new bottle");
			}
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

cv::Point BottleDetection::convertMeasurement(int sensor, Pose pose, int measurement)
{
	
	// position of object in robot frame
	float robot_x = dist_to_robot[sensor][0] 
							+ cos(dist_to_robot[sensor][2])*measurement;
	float robot_y = dist_to_robot[sensor][1] 
							+ sin(dist_to_robot[sensor][2])*measurement;
	
	
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
	int b1_nb_meas_temp = b1.nb_meas;
	b1.position.x = new_x;
	b1.position.y = new_y;
	b1.nb_meas += b2.nb_meas;
	if(b1.nb_meas > BD_NB_MEAS_MAX) {
		b1.nb_meas = BD_NB_MEAS_MAX;
	}
	b1.updated = true;
	
	b2.position.x = new_x;
	b2.position.y = new_y;
	b2.nb_meas += b1_nb_meas_temp;
	if(b2.nb_meas > BD_NB_MEAS_MAX) {
		b2.nb_meas = BD_NB_MEAS_MAX;
	}
	b2.updated = true;
}

void BottleDetection::ageRecordedBottles(void)
{
	int nb_bottles = recorded_bottles.size();
	for(int i=0; i<nb_bottles; i++) {
		// verify if bottle was updated in this cycle
		if(!recorded_bottles[i].updated) {
			recorded_bottles[i].nb_meas -= 1;
			
			// erease bottle if it exists not anymore
			if(recorded_bottles[i].nb_meas <= 0) {
				recorded_bottles.erase(recorded_bottles.begin() + i);
				i -= 1;
				nb_bottles -= 1;
				continue;		
			}
		}
		
		// reset recorded bool
		recorded_bottles[i].updated = false;
	}
}

/*bool BottleDetection::verifyMeasAge(void)
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
}*/
