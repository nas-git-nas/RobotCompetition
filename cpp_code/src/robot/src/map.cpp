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

#include "dm.h"
#include "map.h"

std::vector<cv::Point> Map::getNodes(void){
	return nodes;
}

std::vector<int> Map::getNodePolygon(void)
{
	return node_polygon;
}

void Map::saveRawData(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
	/*for(int i=0; i<msg->data.size(); i++) {
		data[i] = msg->data[i];
	}*/
	for(int i=0; i<MAP_SIZE; i++) {
		for(int j=0; j<MAP_SIZE; j++) {
			map_raw.at<int8_t>(i,j) = msg->data[i*MAP_SIZE+j];
		}
	}
	new_data = true;
	
	if(MAP_VERBOSE_RAW_DATA) {
		std::cout << "map: = "<< std::endl << " " 
					 << map_raw << std::endl << std::endl;
	}
}

bool Map::preprocessData(void)
{
	cv::Mat m_thr = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8UC1, -2);	
	cv::Mat m_dil;
	
	if(MAP_DRAW_MAP) {
		cv::imwrite("map.jpg", map_raw);
	}
	if(MAP_VERBOSE_PREPROCESS) {
		std::cout << "map_raw =" << std::endl << " " << map_raw 
		          << std::endl << std::endl;
	}
	
	// transform map_raw to m: from CV_8SC1 (8-bit signed int with 
	// 1 channel) to CV_8UC1 (unsigned int)
	// and threshold pixels directly
	for(int i=0; i<MAP_SIZE; i++) {
		for(int j=0; j<MAP_SIZE; j++) {
			if(map_raw.at<int8_t>(i,j) == -1) {
				m_thr.at<uint8_t>(i,j) = 255;
			} else if(map_raw.at<int8_t>(i,j)<=PIXEL_THRESHOLD) {
				m_thr.at<uint8_t>(i,j) = 255;
			} else if(map_raw.at<int8_t>(i,j)>PIXEL_THRESHOLD) {
				m_thr.at<uint8_t>(i,j) = 0;
			} else { return false; }
		}
	}
	if(MAP_DRAW_MAP) {
		cv::imwrite("map_threshold.jpg", m_thr);
	}	
	

	
#ifdef DEBUG_FAKE_MAP	
	/* --- for testing without LIDAR --- */
	cv::Mat m = cv::imread("maps/map15.pgm", cv::IMREAD_GRAYSCALE);	
	cv::threshold(m, m_thr, 160, 255, cv::THRESH_BINARY);
	/* ---                           --- */
#endif

	// expand obstacle by robot size
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
									cv::Size(MAP_DILATION_KERNEL,MAP_DILATION_KERNEL));	
	cv::erode(m_thr, m_dil, kernel);
	
	if(MAP_DRAW_MAP) {
		cv::imwrite("map_erode.jpg", m_dil);
	}
	
	map_dilated_robot = m_dil;
	map_preprocessed = m_thr;

	return true;
}

void Map::calcPolygons(cv::Point current_position, 
								 cv::Point destination)
{
	// find contours and approximate them to get polygons
	std::vector<std::vector<cv::Point>> contours;	
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(map_dilated_robot, contours, hierarchy, cv::RETR_LIST,
						  cv::CHAIN_APPROX_NONE);

	std::vector<std::vector<cv::Point>> polygons(contours.size());		
	for(int i=0; i<contours.size(); i++) {
		// approx. contours to get polygons
		cv::approxPolyDP(cv::Mat(contours[i]), polygons[i], 10, true);
	}

	// filter out small polygons	
	int ploygon_size = polygons.size();
	for(int i=0; i<ploygon_size; i++) {
		if(cv::contourArea(polygons[i]) < MAP_POLYGON_MIN_SIZE) {
			polygons.erase(polygons.begin() + i);
			i -= 1;
			ploygon_size -= 1;
		}
	}
	
	if(MAP_VERBOSE_CALC_POLYGONS) {
		std::cout << std::endl << "polygons.size(): " << polygons.size() << std::endl;
		for(int i=0; i<polygons.size(); i++) {
			for(int j=0; j<polygons[i].size(); j++) {
				std::cout << "(" << polygons[i][j].x << "," << polygons[i][j].y << "), ";
			}
			std::cout << std::endl;
			std::cout << "polygon(" << i << ").size: " << cv::contourArea(polygons[i]) << std::endl;
		}
	}
	
	
	if(MAP_DRAW_MAP) {
		cv::Mat m_pol;
		cv::cvtColor(map_preprocessed, m_pol, cv::COLOR_GRAY2BGR);
		cv::drawContours(m_pol, polygons, -1, cv::Scalar(0,0,255), 1);
		cv::imwrite("m_pol.jpg", m_pol);
		map_preprocessed = m_pol;
	}
	
	// create flattened vector of polygons called "nodes" and vector with corresponding polygon
	// index called "node_polygon"
	nodes.clear();
	node_polygon.clear();
	
	nodes.push_back(current_position);
	nodes.push_back(destination);
	node_polygon.push_back(0);
	node_polygon.push_back(1);
	
	for(int i=0; i<polygons.size(); i++) {
		for(int j=0; j<polygons[i].size(); j++) {
			nodes.push_back(polygons[i][j]);
			node_polygon.push_back(i+2); // plus two because of current_position and destination
		}
	}	
	
	return;
}

void Map::draw_graph(std::vector<std::vector<int>> graph, std::vector<int> shortest_path)
{
	if(MAP_DRAW_MAP) {
		cv::Mat m_gra = map_preprocessed;

		// draw visibility graph lines
		for(int i=0; i<graph.size(); i++) { // loop through number of graph points
			for(int j=0; j<i; j++) { // loop through number of connection points
				if(graph[i][j] > 0) {
					cv::line(m_gra, nodes[i], nodes[j], 
								cv::Scalar(255,0,0), 1, cv::LINE_4);
				}
			}
		}
		
		// draw shortest path
		for(int i=0; i<shortest_path.size()-1; i++) {
			cv::line(m_gra, nodes[shortest_path[i]], nodes[shortest_path[i+1]], 
						cv::Scalar(0,255,0), 2, cv::LINE_4);
		}
		
		// draw current position and destination
		cv::circle(m_gra, nodes[0], 5, cv::Scalar(0,0,255), cv::FILLED);
		cv::putText(m_gra, "current pos", nodes[0], cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,255), 1);
		cv::circle(m_gra, nodes[1], 5, cv::Scalar(0,0,255), cv::FILLED);
		cv::putText(m_gra, "destination", nodes[1], cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0,0,255), 1);
		
		cv::imwrite("m_gra.jpg", m_gra);
		map_graph = m_gra;
	}
	return;
}

void Map::printMap(void)
{
	std::cout << "--map start" << std::endl << map_graph << std::endl 
				 << "--map end" << std::endl;
}

