#include "ros/ros.h"
#include <vector>
#include <array>
#include <typeinfo>
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include <cmath>
#include <cstdlib>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#ifndef LOCAL_PATH_PLANNER_H // include header file only once
#define LOCAL_PATH_PLANNER_H

#define VERBOSE_LOCAL_PATH_PLANNER true

#define WHEEL_RADIUS 0.0505 // in m
#define INTER_WHEEL_DISTANCE 0.306 // in m
#define SET_POINT_DISTANCE_THRESHOLD 5 // in cm
#define TURNING_UPPER_THRESHOLD 0.3491 // in rad, 20°
#define TURNING_LOWER_THRESHOLD 0.2618 // in rad, 15°

#define VEL_TURN_MAX 3.5// in rad/s
#define VEL_TURN_MIN 2.5 // in rad/s
#define VEL_TURN_ADJUSTMENT 1.7 // empirical factor to match tunring
#define VEL_TURN_PID_KP 1.0

#define VEL_MOVE_MAX 3.0 // in rad/s
#define VEL_MOVE_BIAS 1.5 // in rad/s
#define VEL_MOVE_PID_KP 1.0
#define VEL_MOVE_PID_KI 0.5

#define PI 3.1415927
#define M2GRID 100 // convert meters to grid (each pixel is one cm)

class LocalPathPlanner
{
   public:
		/*** FUNCTIONS ***/
		void setPoseAndSetPoints(std::vector<cv::Point> trajectory, 
										 float new_heading);
		std::array<float,4> getMotorVelocity(void);
		std::array<float,3> getPose(void);
		std::vector<cv::Point> getSetPoints(void);
        
    
   private:
      /*** VARIABLES ***/
		std::array<float,3> pose = {0,0,0};
		std::vector<cv::Point> set_points;
		std::array<float,4> motor_vel = {0,0,0,0}; // {right front, right back, left front, left back}
		ros::Time time_update_pose;
		bool destination_reached = true;

		/*** FUNCTIONS ***/
		void updateMotorVelocity(void);
		void updatePose(void);
		void robotStop(void);
		void robotMove(float theta_error, float theta_error_integration);
		void robotTurn(float theta_error);
		float limitVelocity(float vel, float max, float min);
		float limitAngle(float angle);
};


#endif // LOCAL_PATH_PLANNER_H
