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
#define MAIN_VERBOSE 					true
#define MAIN_VERBOSE_BD 				false
#define MAIN_VERBOSE_COMMAND 			false

// map.cpp
#define MAP_VERBOSE_RAW_DATA 			false
#define MAP_VERBOSE_PREPROCESS 		false
#define MAP_VERBOSE_CALC_POLYGONS 	false

#define MAP_DRAW_MAP true

#define MAP_SIZE				1200 // in cm, map size in pixels
#define MAP_RESOLUTION 		0.01 // one pixel = 1cm


// visibility.cpp
#define VISIBILITY_VERBOSE 			false

// dijkstra.cpp
#define DIJKSTRA_VERBOSE 				false

// controller.cpp
#define CONTROLLER_VERBOSE 			false
#define CONTROLLER_VERBOSE_IMU 		false
#define CONTROLLER_VERBOSE_POSE_CB 	false
#define CONTROLLER_VERBOSE_HECTOR 	false
#define CONTROLLER_VERBOSE_LOG 		false
#define CONTROLLER_VERBOSE_COMMAND	false
#define CONTROLLER_VERBOSE_MOTORS	false

// lpp.cpp
#define LPP_VERBOSE 						false


// bd.cpp
#define BD_VERBOSE_CALC 				false
#define BD_VERBOSE_SET_ULTRASOUND		true
#define BD_VERBOSE_GET_BOTTLE			false

#define BD_ULTRASOUND_MAX_DISTANCE 60

// dm.cpp
#define DM_VERBOSE 						false
#define DM_VERBOSE_GPP 					false
#define DM_VERBOSE_APPROACH			true
#define DM_VERBOSE_EXPLORE				true
#define DM_VERBOSE_WATCHDOG			true
#define DM_VERBOSE_PICKUP				true
#define DM_VERBOSE_RETURN				true

#define DM_COMMAND_ARM_REST 		0
#define DM_COMMAND_BASKET_REST 	0

#define DM_STATE_IDL 				0
#define DM_STATE_EXPLORE 			1
#define DM_STATE_MOVE				2
#define DM_STATE_APPROACH			3
#define DM_STATE_PICKUP_SEND		4
#define DM_STATE_PICKUP_WAIT		5
#define DM_STATE_PICKUP_VERIFY 	6
#define DM_STATE_RETURN				7
#define DM_STATE_RECYCLE			8
#define DM_STATE_EMPTY_SEND		9
#define DM_STATE_EMPTY_WAIT		10


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
	bool move_arm = false;
	bool move_basket = false;
};

#endif // MAIN_H


