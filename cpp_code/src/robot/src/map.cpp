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
#include "map.h"

std::vector<cv::Point> Map::getNodes(void){
	return nodes;
}

std::vector<int> Map::getNodePolygon(void)
{
	return node_polygon;
}

/*cv::Mat Map::getMapThresholded(void)
{
	return map_thresholded;
}

cv::Mat Map::getMapDilated(void)
{
 return map_dilated_robot;
}*/

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
	cv::Mat m_bot;
	
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

	// expand obstacle by robot size for dilated map
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, 
									cv::Size(MAP_DILATION_KERNEL,MAP_DILATION_KERNEL));	
	cv::erode(m_thr, m_dil, kernel);
	
	// expand obstacle for bottle map
	cv::Mat kernel_bottle = cv::getStructuringElement(cv::MORPH_RECT, 
									cv::Size(MAP_BOTTLE_KERNEL,MAP_BOTTLE_KERNEL));	
	cv::erode(m_thr, m_bot, kernel_bottle);	
	
	if(MAP_DRAW_MAP) {
		cv::imwrite("map_dilated.jpg", m_dil);
		cv::imwrite("map_bottle.jpg", m_bot);
	}
	
	map_thresholded = m_thr;
	map_dilated_robot = m_dil;
	map_bottle = m_bot;

	return true;
}

/*void Map::verifyNewData(cv::Mat new_map)
{
	ROS_INFO_STREAM("---------------------------");
	// return if map_thresholded was not yet created
	if(map_thresholded.empty()) {
		return;
	}

	int nb_same_pixels = 0;
	int nb_different_pixels = 0;
	int nb_black_pixels = 0;
	int nb_negative_false = 0;
	int nb_negative_true = 0;
	
	for(int i=0; i<MAP_SIZE; i++) {
		for(int j=0; j<MAP_SIZE; j++) {			
			if(new_map.at<uint8_t>(i,j)==255 && map_thresholded.at<uint8_t>(i,j)==0) {
				nb_negative_false += 1;
			}
			
			if(new_map.at<uint8_t>(i,j)==0 && map_thresholded.at<uint8_t>(i,j)==255) {
				nb_negative_true += 1;
			}
			
			if(new_map.at<uint8_t>(i,j) == 0) {
				nb_black_pixels += 1;
			}
		}
	}
	
	float ratio_true = float( nb_negative_true ) / float( nb_black_pixels );
	float ratio_false = float( nb_negative_false ) / float(nb_black_pixels );
	
	ROS_INFO_STREAM("Map::verifyNewData: (" << nb_negative_true+nb_negative_false << "," 							<< nb_negative_true << "," << nb_negative_false << ")");
	ROS_INFO_STREAM("Map::verifyNewData: (" << nb_black_pixels << "," << ratio_true 
						 << "," << ratio_false << ")");
}*/

bool Map::calcPolygons(cv::Point current_position, 
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

	// verify area of polygons
	int nb_ploygons = polygons.size();
	int nb_large_polygons = 0;
	for(int i=0; i<nb_ploygons; i++) {
		// calc. area of polygon
		float polygon_area = cv::contourArea(polygons[i]);
		
		// filter out small polygons	
		if(polygon_area < MAP_POLYGON_MIN_SIZE) {
			polygons.erase(polygons.begin() + i);
			i -= 1;
			nb_ploygons -= 1;
		}
		
		// count nb. polygons that are too large
		if(polygon_area > MAP_POLYGON_MAX_SIZE) {
			nb_large_polygons += 1;
			
			if(nb_large_polygons > MAP_MAX_NB_LARGE_POLYGONS) {
				ROS_ERROR("Map::calcPolygons: too many large obstacles");
				//return true;
			}
		}
	}
	
	if(MAP_VERBOSE_CALC_POLYGONS) {
		std::cout << std::endl << "polygons.size(): " << polygons.size() << std::endl;
		for(int i=0; i<polygons.size(); i++) {
			for(int j=0; j<polygons[i].size(); j++) {
				std::cout << "(" << polygons[i][j].x << "," << polygons[i][j].y << "), ";
			}
			std::cout << std::endl;
			std::cout << "polygon(" << i << ").size: " << cv::contourArea(polygons[i]) 
						 << std::endl;
		}
	}
	
	
	if(MAP_DRAW_MAP) {
		cv::Mat m_pol;
		cv::cvtColor(map_thresholded, m_pol, cv::COLOR_GRAY2BGR);
		cv::drawContours(m_pol, polygons, -1, cv::Scalar(0,0,255), 1);
		cv::imwrite("m_pol.jpg", m_pol);
		map_polygons = m_pol;
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
	
	return false;
}

bool Map::verifyBottleMapPoint(cv::Point p, int neighborhood, int threshold)
{
	if(map_bottle.empty()) {
		if(MAP_VERBOSE_PREPROCESS) {
			ROS_WARN("map::verifyBottleMapPoint: map does not exists");
		}
		return true;
	}
	
	// verify how many black pixels are in neighborhood of point p
	int nb_black_pixels = 0;
	for(int k=0; k<2*neighborhood+1; k++) {
	
		// outside map
		if(k<0 || k>MAP_SIZE-1) { continue; }
		
		for( int l=0; l<2*neighborhood+1; l++) {
		
			// outside map
			if(l<0 || l>MAP_SIZE-1) { continue; }
			
			// Invert axes because hector slam and polygons have x-axes in horizontal plane and 
			// the y-axes in the vertical plane. However, for cv::Mat it is the inverse!
			if(unsigned(map_bottle.at<uint8_t>(p.y-neighborhood+k, p.x-neighborhood+l)) == 0) {
				nb_black_pixels += 1;
			}
		}
	}
	
	// verify if number of black pixels exceed threshold
	if(nb_black_pixels >= threshold) {
		
		return true;
	}
	return false;
}

bool Map::verifyDilatedMapPoint(cv::Point p, int neighborhood, int threshold)
{
	if(map_dilated_robot.empty()) {
		if(MAP_VERBOSE_PREPROCESS) {
			ROS_WARN("map::verifyDilatedMapPoint: map does not exists");
		}
		return true;
	}
	
	// verify how many black pixels are in neighborhood of point p
	int nb_black_pixels = 0;
	for(int k=0; k<2*neighborhood+1; k++) {
	
		// outside map
		if(k<0 || k>MAP_SIZE-1) { continue; }
		
		for( int l=0; l<2*neighborhood+1; l++) {
		
			// outside map
			if(l<0 || l>MAP_SIZE-1) { continue; }
			
			// Invert axes because hector slam and polygons have x-axes in horizontal plane and 
			// the y-axes in the vertical plane. However, for cv::Mat it is the inverse!
			if(unsigned(map_dilated_robot.at<uint8_t>(p.y-neighborhood+k, p.x-neighborhood+l)) == 0) {
				nb_black_pixels += 1;
			}
		}
	}
	
	// verify if number of black pixels exceed threshold
	if(nb_black_pixels >= threshold) {
		
		return true;
	}
	return false;
}

void Map::drawNeighborhood(cv::Point p, int neighborhood, int threshold)
{
	cv::Mat m_ngb = map_bottle.clone();
	
	for(int k=0; k<2*neighborhood+1; k++) {
		// outside map
		if(k<0 || k>MAP_SIZE-1) { continue; }	
		for( int l=0; l<2*neighborhood+1; l++) {
			// outside map
			if(l<0 || l>MAP_SIZE-1) { continue; }
			m_ngb.at<uint8_t>(p.y-neighborhood+k, p.x-neighborhood+l) = 100;
		}
	}
	cv::circle(m_ngb, p, 1, cv::Scalar(0,0,50), cv::FILLED);
	
	
	bool on_map = verifyBottleMapPoint(p, neighborhood, threshold);
	cv::putText(m_ngb, std::to_string(on_map), cv::Point(20,20), cv::FONT_HERSHEY_DUPLEX, 
						1, cv::Scalar(0,0,255), 1);
	cv::imwrite("m_ngh.jpg", m_ngb);
}

void Map::draw_graph(std::vector<std::vector<int>> graph, std::vector<int> shortest_path)
{
	if(MAP_DRAW_MAP) {
		cv::Mat map_graph = map_polygons.clone();

		// draw visibility graph lines
		for(int i=0; i<graph.size(); i++) { // loop through number of graph points
			for(int j=0; j<i; j++) { // loop through number of connection points
				if(graph[i][j] > 0) {
					cv::line(map_graph, nodes[i], nodes[j], 
								cv::Scalar(255,0,0), 1, cv::LINE_4);
				}
			}
		}
		
		// draw shortest path
		for(int i=0; i<shortest_path.size()-1; i++) {
			cv::line(map_graph, nodes[shortest_path[i]], nodes[shortest_path[i+1]], 
						cv::Scalar(0,255,0), 2, cv::LINE_4);
		}
		
		// draw current position and destination
		cv::circle(map_graph, nodes[0], 5, cv::Scalar(0,0,255), cv::FILLED);
		cv::putText(map_graph, "current pos", nodes[0], cv::FONT_HERSHEY_DUPLEX, 1,
						 cv::Scalar(0,0,255), 1);
		cv::circle(map_graph, nodes[1], 5, cv::Scalar(0,0,255), cv::FILLED);
		cv::putText(map_graph, "destination", nodes[1], cv::FONT_HERSHEY_DUPLEX, 1,
						 cv::Scalar(0,0,255), 1);
		
		cv::imwrite("m_gra.jpg", map_graph);
	}
	return;
}

void Map::printMap(void)
{
	std::cout << "--map start" << std::endl << map_graph << std::endl 
				 << "--map end" << std::endl;
}


