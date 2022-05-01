
//#include "std_msgs/String.h"

#include <vector>
#include <typeinfo>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>


#include "visibility_graph.h"

std::vector<std::vector<int>> VisibilityGraph::getGraph(void){
	return graph;
}

void VisibilityGraph::calcGraph(std::vector<cv::Point> nodes, std::vector<int> node_polygon)
{
	graph.clear();

	/*nodes = {cv::Point(0,0),cv::Point(1,0),cv::Point(0,1), 
				cv::Point(2,2),cv::Point(2,3),cv::Point(3,3),cv::Point(3,2)};
	node_polygon = {0, 0, 0, 1, 1, 1, 1};*/
									
	/*for(int i=0; i<obstacles.size(); i++) {
		for(int j=0; j<obstacles[i].size(); j++) {
			std::cout << "Point(" << i << "," << j << ") " 
							<< obstacles[i][j] << std::endl;
		}
	}*/
	
	
	
	// loop through all nodes		
	int nb_nodes = 0;
	for(int i=0; i<nodes.size(); i++) {
			
		// add new node to graph
		graph.push_back(graph_point);
		nb_nodes++;

		// loop through all possible edges: ignore top right triangle of node-edge-array 
		// because it is same than bottom left triangle (bi-directional graph)
		int nb_edges = 0;
		for(int j=0; j<i; j++) {
			
			// add new connection
			nb_edges++;					
			bool visible;
			

			// if connection point is in same polygon than initial point
			// define all connection invisible if points are not direct neighboors
			if(node_polygon[i]==node_polygon[j]) { 
				visible = verifyVisibilitySamePolygon(i, j, node_polygon);
				if(VERBOSE_VISIBILITY) {
					std::cout << "(" << i << "," << j << ") = "
									<< visible << " -> same Polygon" << std::endl;
				}	
			}
			// if edge point is on different polygon than node
			// loop through all edges that may prevent visibility	
			else {	
				if(VERBOSE_VISIBILITY) {
					std::cout << "(" << nb_nodes-1 << "," << nb_edges-1 << ") = ";
				}
				
				visible = verifyVisibilityDifferentPolygon(i, j, nodes, node_polygon);
			}
					
			// update graph
			if(visible) { // if points are visible: calc. distance
				graph[nb_nodes-1].push_back(calcDistance(nodes[i],nodes[j])); 
			} else { // if points are not visible: set distance to negative value
				graph[nb_nodes-1].push_back(-1); 
			}							
		} 
	} 

	
	
	if(VERBOSE_VISIBILITY) {
		std::cout << "Graph: " << std::endl;
		for(int i=0; i<graph.size(); i++) {
			for(int j=0; j<graph[i].size(); j++) {
				std::cout << graph[i][j] << " ";
			}
			std::cout << std::endl;
		}
	}
	return;
}


bool VisibilityGraph::verifyVisibilitySamePolygon(int i, int j, std::vector<int> &node_polygon)
{
	if(i == j+1) { // if node is next element after edge of polygon
		return true;
	} else { // if edge is first and node is last element of polygon
		if(polygonLastElement(i, node_polygon)==i && polygonFirstElement(j, node_polygon)==j) {
			return true;
		}
	}
	return false;	
}

bool VisibilityGraph::verifyVisibilityDifferentPolygon(int i, int j, std::vector<cv::Point> 
																		 &nodes, std::vector<int> &node_polygon)
{
	for(int k=0; k<nodes.size(); k++) {								
		// check if polygon k has edges that intersects with edge j
		
		if(i==k || j==k) { continue; }

		// define second point of vertice from polygon iii		
		int l;
		if(polygonLastElement(k, node_polygon)==k) { // if last element of polygon
			l = polygonFirstElement(k, node_polygon);					
		} else { 							// if not last element of polygon
			l = k+1;
		}
							
		// continue if one of points (i,j), (ii,jj) is equel to 
		// (iii,jjj), (iii,jjj_vertice)
		if(i==l || j==l) { continue; }
			
		// check if line between p1 and p2 intersect with line q1 and q2
		if(doIntersect(nodes[i], nodes[j], nodes[k], nodes[l])) {				
			if(VERBOSE_VISIBILITY) {
				std::cout << "false -> intersect = (" << k << "); (" << l << "); " << std::endl;
			}
			return false;									
		}		
	}
		
	if(VERBOSE_VISIBILITY) {
		std::cout << "true -> no intersect" << std::endl;
	}
	return true;		
}

int VisibilityGraph::polygonFirstElement(int i, std::vector<int> &node_polygon)
{
	int index = -1;
	for(int j=0; j<node_polygon.size(); j++) {
		if(node_polygon[j]==node_polygon[i]) {
			index = j;
			break;
		}
	}
	return index;	
}

int VisibilityGraph::polygonLastElement(int i, std::vector<int> &node_polygon)
{
	int index = -1;
	for(int j=0; j<node_polygon.size(); j++) {
		if(node_polygon[j]==node_polygon[i]) {
			index = j;
		}
	}
	return index;	
}

float VisibilityGraph::calcDistance(cv::Point p, cv::Point q)
{
	float delta_x = q.x - p.x;
	float delta_y = q.y - p.y;
	return std::sqrt(delta_x*delta_x + delta_y*delta_y);
}

// Given three collinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool VisibilityGraph::onSegment(cv::Point p, cv::Point q, cv::Point r)
{
	if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
		q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y))
		return true;
	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int VisibilityGraph::orientation(cv::Point p, cv::Point q, cv::Point r)
{
	// See https://www.geeksforgeeks.org/orientation-3-ordered-points/
	// for details of below formula.
	int val = (q.y - p.y) * (r.x - q.x) -
			(q.x - p.x) * (r.y - q.y);

	if (val == 0) return 0; // collinear

	return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool VisibilityGraph::doIntersect(cv::Point p1, cv::Point q1, cv::Point p2, cv::Point q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are collinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and q2 are collinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are collinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are collinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

