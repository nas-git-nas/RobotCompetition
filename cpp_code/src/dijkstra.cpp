
#include <vector>
#include <iostream>

#include "dijkstra.h"


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
 
// A utility function to print the constructed distance array
/*void printSolution(int dist[])
{
	 cout <<"Vertex \t Distance from Source" << endl;
	 for (int i = 0; i < V; i++)
		  cout  << i << " \t\t"<<dist[i]<< endl;
}*/
 
// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
std::vector<int> Dijkstra::calcPath(std::vector<std::vector<int>> graph, int start, int goal)
{	


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
		// Pick the minimum distance vertex from the set of vertices not
		// yet processed. u is always equal to src in the first iteration.
		int u = minDistance(dist, sptSet, graph.size());

		// Mark the picked vertex as processed
		sptSet[u] = true;

		
		// Update dist value of the adjacent vertices of the picked vertex.
		for (int v = 0; v < graph.size(); v++) {
			// the graph is bi-directional and only the upper right triangle is filled out
			int vertice_dist; // distance between u and v
			if(u>=v) { // lower left triangle
				vertice_dist = graph[u][v];
			} else { // upper right triangle
				vertice_dist = graph[v][u];
			}
			std::cout << "(u=" << u << ",v=" << v <<"), dist=" << vertice_dist;
			std::cout << "sptSet=" << sptSet[v] << std::endl;
			
			// Update dist[v] only if is not in sptSet, there is an edge from
			// u to v, and total weight of path from src to  v through u is
			// smaller than current value of dist[v]
			if (!sptSet[v] && vertice_dist>0 && dist[u] != DISTANCE_MAX
				 && dist[u] + vertice_dist < dist[v]) {
				 dist[v] = dist[u] + vertice_dist;
				 prev[v] = u;
			}
		}
	}
	
	// calc. shortest path
	std::vector<int> shortest_path; // shortest path from start to goal
	shortest_path.push_back(start);
	int current = start;
	while(current != goal) {
		shortest_path.push_back(prev[current]);
		current = prev[current];
	}

	// print the constructed distance array
	//printSolution(dist);
	
	//std::vector<int> shortest_path;
	return shortest_path;
}

