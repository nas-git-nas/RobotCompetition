#include <vector>
#include <iostream>


#ifndef DIJKSTRA_H // include header file only once
#define DIJKSTRA_H


#define DISTANCE_MAX 10000 //very large number, should be larger than the largest distance
#define VERBOSE_DIJKSTRA true

class Dijkstra
{
	public:
		/*** FUNCTIONS ***/
		std::vector<int> getShortestPath(void);
		
		void calcPath(std::vector<std::vector<int>> graph);
	
	private:
		/*** VARIABLES ***/
		std::vector<int> shortest_path; // shortest path from start to goal
	
		/*** FUNCTIONS ***/
		// A utility function to find the vertex with minimum distance value, from
		// the set of vertices not yet included in shortest path tree
		int minDistance(int dist[], bool sptSet[], int graph_size);	
};



#endif // DIJKSTRA_H


