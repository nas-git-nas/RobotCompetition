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

#define LPP_USE_IMU_TO_UPDATE_HEADING false

#define WHEEL_RADIUS 0.0505 // in m
#define INTER_WHEEL_DISTANCE 0.306 // in m
#define SET_POINT_DISTANCE_THRESHOLD 8 // in cm, error allowed to reach setpoint
#define TURNING_UPPER_THRESHOLD 0.3491 // in rad, 20°
#define TURNING_LOWER_THRESHOLD 0.2618 // in rad, 15°

#define LPP_BOTTLE_DIST_THR 5 // in cm
#define LPP_BOTTLE_ANGLE_THR 0.0873 // rad, equal to 5 degrees
#define APPROACH_UPPER_THRESHOLD 0.1745 // in rad, 10°
#define APPROACH_LOWER_THRESHOLD 0.0873 // in rad, 5°
#define LPP_ARM_LENGTH 35 // in cm

#define LPP_NB_SENSORS 7
#define LPP_MEAS_HEADING_ANGLE 0.0873 // rad, turn 10 degrees to correct heading 
#define LPP_MEAS_HEADING_THR 30 // in cm, above this value measurement is not taken into
										 // account when doing heading correction
#define VEL_TURN_ADJUSTMENT 1.2 // empirical factor to match tunring
#define VEL_MOVE_ADJUSTMENT 0.8

#define LPP_MOVE_VEL_MAX 2.5 // in rad/s
#define LPP_MOVE_VEL_MIN 2.0 // in rad/s
#define LPP_MOVE_DIST_MAX 100.0 // in cm
#define LPP_MOVE_DIST_MIN 10.0 // in cm
#define LPP_MOVE_DELTA ((LPP_MOVE_VEL_MAX-LPP_MOVE_VEL_MIN)/(LPP_MOVE_DIST_MAX-LPP_MOVE_DIST_MIN))
#define LPP_MOVE_ZERO (LPP_MOVE_VEL_MIN - (LPP_MOVE_DELTA*LPP_MOVE_DIST_MIN))
#define LPP_MOVE_BIAS_MAX 0.15 // in %, max. velocity bias depending on error in heading
#define LPP_MOVE_BIAS_DELTA (LPP_MOVE_BIAS_MAX/LPP_TURN_ANGLE_MIN)

#define LPP_APPR_VEL_MAX 1.2 // in rad/s
#define LPP_APPR_VEL_MIN 0.8 // in rad/s
#define LPP_APPR_DIST_MAX 50.0 // in cm
#define LPP_APPR_DIST_MIN 10.0 // in cm
#define LPP_APPR_DELTA ((LPP_APPR_VEL_MAX-LPP_APPR_VEL_MIN)/(LPP_APPR_DIST_MAX-LPP_APPR_DIST_MIN))
#define LPP_APPR_ZERO (LPP_APPR_VEL_MIN - (LPP_APPR_DELTA*LPP_APPR_DIST_MIN))
#define LPP_APPR_BIAS_MAX 0.2 // in %, max. velocity bias depending on error in heading
#define LPP_APPR_BIAS_DELTA (LPP_APPR_BIAS_MAX/LPP_TURN_ANGLE_MIN)

#define LPP_TURN_VEL_MAX 0.7 // in rad/s
#define LPP_TURN_VEL_MIN 0.5 // in rad/s
#define LPP_TURN_ANGLE_MAX 1.571 // in rad, 90°
#define LPP_TURN_ANGLE_MIN 0.2618 // in rad, 15°
#define LPP_TURN_DELTA ((LPP_TURN_VEL_MAX-LPP_TURN_VEL_MIN)/(LPP_TURN_ANGLE_MAX-LPP_TURN_ANGLE_MIN))
#define LPP_TURN_ZERO (LPP_TURN_VEL_MIN - (LPP_TURN_DELTA*LPP_TURN_ANGLE_MIN))

#define LPP_CURVE_VEL_MAX 1.0 // in rad/s
#define LPP_CURVE_VEL_MIN 0.4 // in rad/s

#define DEGREE2RAD 0.017453
#define RAD2DEGREE 57.296
#define M2GRID 100 // convert meters to grid (each pixel is one cm)

class LPP
{
   public:
		/*** FUNCTIONS ***/
		void setPose(float *new_pose);
		void setSetPoints(std::vector<cv::Point> trajectory);
		void setIMUData(float *gyro_data);
		void stopMotors(void);
		void set_dm_state(uint8_t new_state);
		void set_meas(std::array<int,LPP_NB_SENSORS> new_meas);
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
		bool stop_robot = true;
		float gyro[3] = {0,0,0};
		uint8_t dm_state;
		std::array<int,LPP_NB_SENSORS> meas;
		
		/*** FUNCTIONS ***/
		void updateMotorVelocity(void);
		void updateApproachVelocity(void);
		void updatePose(void);
		void robotStop(void);
		void robotMove(float theta_error, float distance);
		void robotApproach(float theta_error, float distance);
		void robotTurn(float theta_error);
		void robotCurve(float theta_error);
		float limitVelocity(float vel, float max, float min);
		float limitAngle(float angle);
};


#endif // LOCAL_PATH_PLANNER_H

/*
#define VEL_TURN_MAX 1.2 // 3.5// in rad/s
#define VEL_TURN_MIN 0.6 // in rad/s

#define VEL_TURN_PID_KP 1.0

#define VEL_MOVE_MAX 3.0 // in rad/s
#define VEL_MOVE_MIN 1.5 // in rad/s
#define VEL_MOVE_BIAS 2.5 // in rad/s
#define VEL_MOVE_PID_KP 1.0
#define VEL_MOVE_PID_KI 0.0
*/
