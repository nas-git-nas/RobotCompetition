
#include <vector>
#include <iostream>

#include "dijkstra.h"



std::vector<int> Dijkstra::getShortestPath(void)
{
	return shortest_path;
}

 
// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
void Dijkstra::calcPath(std::vector<std::vector<int>> graph)
{	
	int start = 2; // by definition first element of nodes and therefore also of graph
	int goal = 4; // by definition second element of nodes and therefore also of graph

	std::cout << "Graph2: " << std::endl;
	for(int i=0; i<graph.size(); i++) {
		for(int j=0; j<graph[i].size(); j++) {
			std::cout << graph[i][j] << " ";
		}
		std::cout << std::endl;
	}
	
	
	int dist[graph.size()]; // dist[i] will hold the shortest distance from start to i
	
	int prev[graph.size()]; // prev[i] contains previous node of shortest path from node i
 	
	// sptSet[i] is true if vertex i is included in shortest path tree 
	// or shortest distance from src to i is finalized
	bool sptSet[graph.size()];
	
	// Initialize all distances as INFINITE and stpSet[] as false
	for (int i = 0; i < graph.size(); i++) {
		dist[i] = DISTANCE_MAX;
		sptSet[i] = false;
		prev[i] = -1; // untrue value
	}
	
	// Distance of source vertex from itself is always 0
	dist[goal] = 0;

	
	// Find shortest path for all vertices
	for (int count = 0; count < graph.size() - 1; count++) {
	
		std::cout << "count=" << count <<  ", graph.size()" << graph.size() << std::endl;
	
		// Pick the minimum distance vertex from the set of vertices not
		// yet processed. u is always equal to src in the first iteration.
		int u = minDistance(dist, sptSet, graph.size());

		// Mark the picked vertex as processed
		sptSet[u] = true;

		
		// Update dist value of the adjacent vertices of the picked vertex.
		std::cout << "u=" << u <<  ", graph.size()" << graph.size() << std::endl;
		for (int v = 0; v < u; v++) {
			
			if(VERBOSE_DIJKSTRA) {
				std::cout << "(u=" << u << ",v=" << v <<"), graph[u][v]=" << graph[u][v];
				std::cout << " sptSet=" << sptSet[v] << " dist[u]=" << dist[u] 
							 << " dist[v]=" << dist[v] << " prev[v]=" << prev[v] << std::endl;
			}
			
			// Update dist[v] only if is not in sptSet, there is an edge from
			// u to v, and total weight of path from src to  v through u is
			// smaller than current value of dist[v]
			if (!sptSet[v] && graph[u][v]>0 && dist[u] != DISTANCE_MAX
				 && dist[u] + graph[u][v] < dist[v]) {
				 dist[v] = dist[u] + graph[u][v];
				 prev[v] = u;
			}
		}
	}
	
	std::cout << "after loop" << std::endl;
	
	// calc. shortest path
	shortest_path.clear();
	
	shortest_path.push_back(start);	
	int current = start;
	
	while(current != goal) {
		std::cout << "current=" << current << ", prev=" << prev[current] << std::endl;
		shortest_path.push_back(prev[current]);
		current = prev[current];
	}

	return;
}

// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int Dijkstra::minDistance(int dist[], bool sptSet[], int graph_size)
{
	
	// Initialize min value
	int min = DISTANCE_MAX;
	int min_index;

	for (int v = 0; v < graph_size; v++)
		if (sptSet[v] == false && dist[v] <= min) {
			min = dist[v];
			min_index = v;
		}
	return min_index;
}
