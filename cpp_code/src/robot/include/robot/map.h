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

#ifndef MAP_H // include header file only once
#define MAP_H


#define PIXEL_THRESHOLD 60 // pixel oppucancy certainty: [0,100]

#define MAP_BRICK_LENGTH 30 // cm
#define MAP_BRICK_WIDTH 15 // cm
#define MAP_DILATION_KERNEL 60 // cm, dilation/expansion of obstacles
#define MAP_BOTTLE_KERNEL 30 // cm, dilation of obstacles for bottle map

#define MAP_POLYGON_MIN_SIZE 200 // cm, min. size of polygon to be not ignored
#define MAP_POLYGON_MAX_SIZE ((2*MAP_DILATION_KERNEL+MAP_BRICK_LENGTH) \
										*(2*MAP_DILATION_KERNEL+MAP_BRICK_WIDTH))
#define MAP_MAX_NB_LARGE_POLYGONS 100 //TODO: define value



// class containing map
class Map
{
	public:
		/*** VARIABLES ***/
		std::vector<int8_t> data = std::vector<int8_t>(MAP_SIZE*MAP_SIZE, -2);
		bool new_data = false;	
		
		/*** FUNCTIONS ***/
		std::vector<cv::Point> getNodes(void);
		std::vector<int> getNodePolygon(void);
		//cv::Mat getMapThresholded(void);
		//cv::Mat getMapDilated(void);
		void saveRawData(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		
		bool preprocessData(void);
		
		bool calcPolygons(cv::Point current_position, 
								 cv::Point destination);
		bool verifyBottleMapPoint(cv::Point p, int neighborhood, int threshold);
		bool verifyDilatedMapPoint(cv::Point p, int neighborhood, int threshold);
		void drawNeighborhood(cv::Point p, int neighborhood, int threshold);
		void draw_graph(std::vector<std::vector<int>> graph, 
								std::vector<int> shortest_path);
		void printMap(void);

							 
	private:
		/*** VARIABLES ***/
		cv::Mat map_raw = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8SC1, -2);
		cv::Mat map_thresholded;
		cv::Mat map_dilated_robot;
		cv::Mat map_bottle;
		cv::Mat map_polygons;
		cv::Mat map_graph;
		
		std::vector<cv::Point> nodes;
		std::vector<int> node_polygon;
		
		/*** FUNCTIONS ***/
		void verifyNewData(cv::Mat new_map);

};

#endif // MAP_H
