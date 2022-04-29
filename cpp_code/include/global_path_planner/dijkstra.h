#include <vector>
#include <iostream>


#ifndef DIJKSTRA_H // include header file only once
#define DIJKSTRA_H


#define DISTANCE_MAX 10000 //very large number, should be larger than the largest distance

class Dijkstra
{
public:

	// A utility function to find the vertex with minimum distance value, from
	// the set of vertices not yet included in shortest path tree
	int minDistance(int dist[], bool sptSet[], int graph_size);
	
	std::vector<int> calcPath(std::vector<std::vector<int>> graph, int start, int goal);
	
};



#endif // DIJKSTRA_H


