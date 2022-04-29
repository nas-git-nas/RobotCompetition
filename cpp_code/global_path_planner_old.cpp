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


#define VERBOSE 	false
#define VERBOSE_VISIBILITY 	true
#define MAP_SIZE	100

// class containing map
class Map
{
	public:
		std::vector<int8_t> data = std::vector<int8_t>(MAP_SIZE*MAP_SIZE, -2);
		bool new_data = false;
		cv::Mat matrix = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8S, -2);
		cv::Mat matrix_dilated = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8S, -2);
		cv::Mat matrix_threshold = cv::Mat(MAP_SIZE, MAP_SIZE, CV_8S, -2);
		
		void saveRawData(const nav_msgs::OccupancyGrid::ConstPtr& msg)
		{
			/*for(int i=0; i<msg->data.size(); i++) {
				data[i] = msg->data[i];
			}*/
			for(int i=0; i<MAP_SIZE; i++) {
				for(int j=0; j<MAP_SIZE; j++) {
					matrix.at<int8_t>(i,j) = msg->data[i*MAP_SIZE+j];
				}
			}
			new_data = true;
			
			if(VERBOSE) {
				std::cout << "map: = "<< std::endl << " " 
							 << matrix << std::endl << std::endl;
			}
		}
		
		std::vector<std::vector<cv::Point>> preprocessData(void)
		{
			//cv::Mat kernel = cv::Mat(3, 3, CV_8S, 1);
			

			cv::Mat mat = cv::imread("maps/map15.pgm", cv::IMREAD_GRAYSCALE);
			cv::Mat mat_thr;
			cv::Mat mat_dil;
			std::vector<std::vector<cv::Point>> contours;
			
			std::vector<cv::Vec4i> hierarchy;
			cv::Mat mat_cont;



			cv::threshold(mat, mat_thr, 160, 255, cv::THRESH_BINARY);
			
			cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11,11));
			//cv::dilate(mat_thr, mat_dil, kernel);
			cv::erode(mat_thr, mat_dil, kernel);
			cv::imwrite("test4.jpg", mat_dil);
			
			cv::findContours(mat_dil, contours, hierarchy, cv::RETR_LIST, 
									cv::CHAIN_APPROX_NONE);
			
			std::vector<std::vector<cv::Point>> polygons(contours.size());					
			for(int i=0; i<contours.size(); i++) {
				cv::approxPolyDP(cv::Mat(contours[i]), polygons[i], 10, true);
			}
									
			cv::Scalar color(0,0,150);
			cv::drawContours(mat_thr, polygons, -1, color, 1);
			
			/*for(int i=0; i<polygons.size(); i++) {
				for(int j=0; j<polygons[i].size(); j++) {
					std::cout << "Point(" << i << "," << j << ") " 
									<< polygons[i][j] << std::endl;
				}
			}*/

			cv::imwrite("test.jpg", mat);
			cv::imwrite("test2.jpg", mat_thr);
			cv::imwrite("test3.jpg", mat_dil);
			
			return polygons;
		}
		
} map;

class VisibilityGraph
{
	public:
		std::vector<std::vector<cv::Point>> obstacles;
		std::vector<std::vector<int>> graph;
		std::vector<int> graph_point;
		
		void calcGraph(std::vector<std::vector<cv::Point>> &polygons)
		{
			obstacles = polygons;
			graph.clear();

			/*obstacles = { {cv::Point(0,0),cv::Point(1,0),cv::Point(0,1)},
								{cv::Point(2,2),cv::Point(2,3),cv::Point(3,3),cv::Point(3,2)} };*/
											
			/*for(int i=0; i<obstacles.size(); i++) {
				for(int j=0; j<obstacles[i].size(); j++) {
					std::cout << "Point(" << i << "," << j << ") " 
									<< obstacles[i][j] << std::endl;
				}
			}*/
			
			
			
			// loop through all initial points			
			int nb_points = 0;
			for(int i=0; i<obstacles.size(); i++) {
				for(int j=0; j<obstacles[i].size(); j++) {
					
					// add new point to graph
					graph.push_back(graph_point);
					nb_points++;

					// loop through all possible connection points
					int nb_connections = 0;
					for(int ii=0; ii<obstacles.size(); ii++) {
						for(int jj=0; jj<obstacles[ii].size(); jj++) {
						
							// add new connection
							nb_connections++;					
							bool visible;
							
							// skip connection point if it is same than initial point or 
							// inverse visibility was already calc.
							if(nb_points >= nb_connections) {
								visible = false;
								
								if(VERBOSE_VISIBILITY) {
									std::cout << "(" << nb_points-1 << "," << nb_connections-1 << ") = "
													<< visible << " -> lower triangle" << std::endl;				
								}					
							}
							// if connection point is in same polygon than initial point
							// define all connection invisible if points are not direct neighboors
							else if(i==ii) { 
								visible = verifyVisibilitySamePolygon(j, ii, jj);
								if(VERBOSE_VISIBILITY) {
									std::cout << "(" << nb_points-1 << "," << nb_connections-1 << ") = "
													<< visible << " -> same Polygon" << std::endl;				
								}	
							}
							// if connection point is on different polygon than initial point
							// loop through all vertices that may prevent visibility	
							else {	
								if(VERBOSE_VISIBILITY) {
									std::cout << "(" << nb_points-1 << "," << nb_connections-1 << ") = ";	
								}
								
								visible = verifyVisibilityDifferentPolygon(i, j, ii, jj);
							}
									
							// update graph
							if(visible) { // if points are visible: calc. distance
								graph[nb_points-1].
										push_back(calcDistance(obstacles[i][j],obstacles[ii][jj])); 
							} else { // if points are not visible: set distance to negative value
								graph[nb_points-1].push_back(-1); 
							}							
						} 
					} // end of for loops: connection points
				}
			} // end of for loops: initial points
			
			
			
			std::cout << "Graph: " << std::endl;
			for(int i=0; i<graph.size(); i++) {
				for(int j=0; j<graph[i].size(); j++) {
					std::cout << graph[i][j] << " ";
				}
				std::cout << std::endl;
			}		
		}
		
		bool verifyVisibilitySamePolygon(int j, int ii, int jj)
		{
			if(jj == j+1) {
				return true;
			} else if(jj==obstacles[ii].size()-1 && j==0) {
				return true;
			}
			return false;	
		}
		
		bool verifyVisibilityDifferentPolygon(int i, int j, int ii, int jj)
		{
			for(int iii=0; iii<obstacles.size(); iii++) {								
				// check if polygon iii has vertice that intersects with line
				// between points j and jj (from polygons i and ii)
				std::cout << obstacles[iii].size();
				for(int jjj=0; jjj<obstacles[iii].size(); jjj++) {
					// define second point of vertice from polygon iii
					
					int jjj_vertice;
					if(jjj==obstacles[iii].size()-1) { // if last element of polygon
						jjj_vertice = 0;
					} else { 							// if not last element of polygon
						jjj_vertice = jjj+1;
					}
										
					// continue if one of points (i,j), (ii,jj) is equel to 
					// (iii,jjj), (iii,jjj_vertice)
					if((i==iii && (j==jjj || j==jjj_vertice)) || 
						(ii==iii && (jj==jjj || jj==jjj_vertice))) { continue; }
						
				
			
					// check if line between p1 and p2 intersect with line q1 and q2
					if(doIntersect(obstacles[i][j], obstacles[ii][jj], 
									obstacles[iii][jjj], obstacles[iii][jjj_vertice])) {
						
						if(VERBOSE_VISIBILITY) {
							std::cout << "false -> intersect = (" << iii << "," << jjj << "); "
										<< iii << "," <<jjj_vertice <<"); " << std::endl;
						}
						return false;									
					}		
				}
			} // end of for loops: vertices 
			
			if(VERBOSE_VISIBILITY) {
				std::cout << "true -> no intersect" << std::endl;
			}
			return true;		
		}
		
		float calcDistance(cv::Point p, cv::Point q)
		{
			float delta_x = q.x - p.x;
			float delta_y = q.y - p.y;
			return std::sqrt(delta_x*delta_x + delta_y*delta_y);
		}

		// Given three collinear points p, q, r, the function checks if
		// point q lies on line segment 'pr'
		bool onSegment(cv::Point p, cv::Point q, cv::Point r)
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
		int orientation(cv::Point p, cv::Point q, cv::Point r)
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
		bool doIntersect(cv::Point p1, cv::Point q1, cv::Point p2, cv::Point q2)
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
	
} visibility_graph;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//std::string s(msg->data.begin(), msg->data.end());
	//ROS_INFO("I heard: [%s]", s.c_str());
	
	map.saveRawData(msg);


	/*if(VERBOSE) {
		int j = 0;
		for(int i:map.data) {
			std::cout<<"j : " << j;
			std::cout<<" occupancy = " << i << "\n";
			j++;
		}
	}*/
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "global_path_planner");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("map", 100, mapCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  int counter = 0;
  while(true) {
  		while(!map.new_data) {
  			ros::spinOnce();
  		}
  		map.new_data = false;
  		
  		std::vector<std::vector<cv::Point>> polygons = map.preprocessData();
  		
		visibility_graph.calcGraph(polygons);

  		
  		std::cout<<" spinOnce: " << counter << "\n";
  		counter++;
  		ros::Duration(1, 0).sleep();
	}
  return 0;
}
