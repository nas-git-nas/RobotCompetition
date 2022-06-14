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
#include "map.h"

void BottleDetection::setUltrasound(std::array<int,BD_NB_SENSORS> meas, Map map, 
												Pose pose)
{
	// verify if measurements are false and save it
	for(int i=0; i<BD_NB_SENSORS; i++) {
		// mask measurments if they are out of range
		if(meas[i]>max_meas_distance[i] || meas[i]<0) {
			meas[i] = 0;
			if(BD_VERBOSE_SET_ULTRASOUND) {
				ROS_WARN("main::arduinoCB: measurement out of range");
			}
		}
		bottle_meas[i] = 0; // reset array
	}

	// calc. position of measured bottles
	std::vector<Bottle> new_bottles = calcNewBottles(map, pose, meas);
	
	// add new bottles to recorded bottles vector and update it
	addNewBottles(new_bottles);
	
	//	subtract one measurement number of all bottles that were not updated this round
	ageRecordedBottles(pose);
	
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

Bottle BottleDetection::getBestBottle(Map map)
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

	// verify if best bottle is inside an obstacle
	if(map.verifyBottleMapPoint(recorded_bottles[most_meas_index].position, BD_SEARCH_NEIGHBORHOUD,
									 BD_SEARCH_NB_PIXEL_THR)) {
		Bottle nothing;
		nothing.position.x = 0;
		nothing.position.y = 0;
		nothing.nb_meas = 0;
		
		if(BD_VERBOSE_GET_BOTTLE) {
			ROS_INFO("bd::getBestBottle: bottle is inside obstacle!");
		}
		return nothing;
	}


	return recorded_bottles[most_meas_index];
}

std::array<int,BD_NB_SENSORS> BottleDetection::getBottleMeas(void)
{
	return bottle_meas;
}

void BottleDetection::clearRecordedBottles(void)
{
	recorded_bottles.clear();
}

std::vector<Bottle> BottleDetection::calcNewBottles(Map map, Pose pose,
																	 std::array<int,BD_NB_SENSORS> meas)
{
	std::vector<Bottle> bottles;

	
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

		//ROS_INFO_STREAM("bd::calcBottlePosition: sensor[" << i << "] = (" << object.x << "," 
		//							<< object.y << ")");

		
		
		// verify if object is an obstacle
		if(map.verifyBottleMapPoint(object, BD_SEARCH_NEIGHBORHOUD, BD_SEARCH_NB_PIXEL_THR)) {
			if(BD_VERBOSE_CALC) {
				ROS_INFO_STREAM("bd::calcBottlePosition: object is obstacle");
			}
			return bottles;
		}
		
		if(BD_VERBOSE_CALC) {
			ROS_INFO_STREAM("bd::calcBottlePosition: object is bottle");
		}
		
		// add object to bottles vector if it is not on map
		Bottle temp_bottle;
		temp_bottle.position = object;
		temp_bottle.nb_meas = 2;
		temp_bottle.updated = true;
		bottles.push_back(temp_bottle);
		
		// save measurment if it is no obstacle
		bottle_meas[i] = meas[i]; 
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

void BottleDetection::ageRecordedBottles(Pose pose)
{
	int nb_bottles = recorded_bottles.size();
	for(int i=0; i<nb_bottles; i++) {

		// verify if bottle is in the dead angle between sensors on the side 
		// and sensors in the front
		/*int error_x = recorded_bottles[i].position.x - pose.position.x;
		int error_y = recorded_bottles[i].position.y - pose.position.y;
		float theta_error = abs(atan2f(error_y, error_x) - pose.heading);
		if(BD_AGE_DEAD_ANGLE_MIN < theta_error &&  theta_error < BD_AGE_DEAD_ANGLE_MAX) {
			recorded_bottles[i].updated = false;
			ROS_ERROR("bd::ageRecordedBottles: do not age !");
			continue;
		}*/

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
		//ROS_ERROR("bd::ageRecordedBottles: age -----------------");
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
