#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>



#ifndef MAIN_H // include header file only once
#define MAIN_H

// debugging
//#define DEBUG_FAKE_MAP
//#define DEBUG_FAKE_MEAS
//#define DEBUG_WITHOUT_LPP

// main.cpp
#define MAIN_VERBOSE 			false
#define MAIN_VERBOSE_BD 		false
#define MAIN_VERBOSE_COMMAND 	false

// map.cpp
#define MAP_VERBOSE_RAW_DATA 			false
#define MAP_VERBOSE_PREPROCESS 		false
#define MAP_VERBOSE_CALC_POLYGONS 	false
#define MAP_DRAW_MAP 					true

#define MAP_SIZE				1000 // in cm, map size in pixels
#define MAP_RESOLUTION 		0.01 // one pixel = 1cm
#define MAP_START_X 			0.09 // starting position in MAP_SIZE precentage
#define MAP_START_Y 			0.09

// visibility.cpp
#define VISIBILITY_VERBOSE false

// dijkstra.cpp
#define DIJKSTRA_VERBOSE false

// controller.cpp
#define CONTROLLER_VERBOSE 			false
#define CONTROLLER_VERBOSE_IMU 		false
#define CONTROLLER_VERBOSE_POSE_CB 	false
#define CONTROLLER_VERBOSE_HECTOR 	false

// lpp.cpp
#define LPP_VERBOSE false

// bd.cpp
#define BD_VERBOSE false

#define BD_ULTRASOUND_MAX_DISTANCE 30

// dm.cpp
#define DM_VERBOSE 			false
#define DM_VERBOSE_GPP 		true
#define DM_VERBOSE_BD		false

#define DM_COMMAND_ARM_REST 		0
#define DM_COMMAND_BASKET_REST 	0

#define DM_STATE_IDL 		0
#define DM_STATE_EXPLORE 	1
#define DM_STATE_APPROACH	2
#define DM_STATE_PICKUP		3
#define DM_STATE_RETURN		4
#define DM_STATE_EMPTY		5


// general
#define PI 3.141593


// structures
struct Pose {
	cv::Point position;
	float heading = 0;
};

struct Command {
	uint8_t dm_state;
	std::vector<uint16_t> trajectory_x;
	std::vector<uint16_t> trajectory_y;
	int nb_nodes;
	bool stop_motor = true;
	float arm_angle;
	float basket_angle;
	bool air_pump = false;
};

#endif // MAIN_H


