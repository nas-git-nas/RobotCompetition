
#include <vector>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>


#ifndef VISIBILITY_GRAPH_H // include header file only once
#define VISIBILITY_GRAPH_H



class VisibilityGraph
{
	public:

		/*** FUNCTIONS ***/
		std::vector<std::vector<int>> getGraph(void);
		
		void calcGraph(std::vector<cv::Point> nodes, std::vector<int> node_polygon);
															 
	private:
		/*** VARIABLES ***/
		std::vector<std::vector<int>> graph;
		std::vector<int> graph_point;
		
		/*** FUNCTIONS ***/		
		bool verifyVisibilitySamePolygon(int i, int j, std::vector<int> &node_polygon);
		
		bool verifyVisibilityDifferentPolygon(int i, int j, std::vector<cv::Point> &nodes, 
															std::vector<int> &node_polygon);
		
		int polygonFirstElement(int i, std::vector<int> &node_polygon);
		
		int polygonLastElement(int i, std::vector<int> &node_polygon);
		
		float calcDistance(cv::Point p, cv::Point q);

		// Given three collinear points p, q, r, the function checks if
		// point q lies on line segment 'pr'
		bool onSegment(cv::Point p, cv::Point q, cv::Point r);

		// To find orientation of ordered triplet (p, q, r).
		// The function returns following values
		// 0 --> p, q and r are collinear
		// 1 --> Clockwise
		// 2 --> Counterclockwise
		int orientation(cv::Point p, cv::Point q, cv::Point r);

		// The main function that returns true if line segment 'p1q1'
		// and 'p2q2' intersect.
		bool doIntersect(cv::Point p1, cv::Point q1, cv::Point p2, cv::Point q2);
	
};

#endif // VISIBILITY_GRAPH_H
