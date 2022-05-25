#include <vector>
#include <iostream>

#include "map.h"


#ifndef DM_H // include header file only once
#define DM_H

// debugging
#define DEBUG_FAKE_MAP
#define DEBUG_WITHOUT_LPP

// dm.cpp
#define DM_VERBOSE_GPP false

// map.cpp
#define MAP_VERBOSE_RAW_DATA false
#define MAP_VERBOSE_PREPROCESS false
#define MAP_VERBOSE_CALC_POLYGONS false
#define MAP_DRAW_MAP true

// visibility.cpp
#define VISIBILITY_VERBOSE 	false

// dijkstra.cpp
#define DIJKSTRA_VERBOSE false

// lpp_node.cpp
#define LPP_NODE_VERBOSE true

// lpp.cpp
#define LPP_VERBOSE false

// bd.cpp
#define BD_VERBOSE false





// global variables

Map map;


struct Pose {
	cv::Point position;
	float heading = 0;
} pose;

#endif // DM_H


