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

#ifndef MAP_H // include header file only once
#define MAP_H

#define MAP_SIZE	100
#define VERBOSE_MAP false
#define SAVE_MAP true

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
		void saveRawData(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		
		std::vector<std::vector<cv::Point>> preprocessData(void);
		
		void draw_graph(std::vector<std::vector<int>> &graph, 
							 std::vector<int> &shortest_path,
							 std::vector<std::vector<cv::Point>> &polygons);

							 
	private:
		/*** VARIABLES ***/
		cv::Mat map_raw = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8S, -2);
		cv::Mat map_preprocessed;
		cv::Mat map_graph;
		
		std::vector<cv::Point> nodes;
		std::vector<int> node_polygon;
		
		/*** FUNCTIONS ***/

};

#endif // MAP_H
