#include <vector>
#include <iostream>

#include "map.h"


#ifndef MAIN_H // include header file only once
#define MAIN_H

// debugging
#define DEBUG_FAKE_MAP
//#define DEBUG_FAKE_MEAS
#define DEBUG_WITHOUT_LPP

// main.cpp
#define MAIN_VERBOSE false
#define MAIN_VERBOSE_GPP false
#define MAIN_VERBOSE_BD false

// map.cpp
#define MAP_VERBOSE_RAW_DATA false
#define MAP_VERBOSE_PREPROCESS false
#define MAP_VERBOSE_CALC_POLYGONS false
#define MAP_DRAW_MAP true

// visibility.cpp
#define VISIBILITY_VERBOSE 	false

// dijkstra.cpp
#define DIJKSTRA_VERBOSE false

// controller.cpp
#define CONTROLLER_VERBOSE false
#define CONTROLLER_VERBOSE_IMU false
#define CONTROLLER_VERBOSE_POSE_CB false
#define CONTROLLER_VERBOSE_HECTOR false

// lpp.cpp
#define LPP_VERBOSE false

// bd.cpp
#define BD_VERBOSE false

// dm.cpp
#define DM_COMMAND_ARM_REST 0
#define DM_COMMAND_BASKET_REST 0

#define PI 3.141593


// structures
struct Pose {
	cv::Point position;
	float heading = 0;
};

struct Command {
	std::vector<uint16_t> trajectory_x;
	std::vector<uint16_t> trajectory_y;
	int nb_nodes;
	bool stop_motor = true;
	float arm_angle;
	float basket_angle;
	bool air_pump = false;
};

#endif // MAIN_H


