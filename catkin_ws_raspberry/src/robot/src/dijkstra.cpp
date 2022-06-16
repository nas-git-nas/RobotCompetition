
#include <vector>
#include <iostream>

#include "main.h"
#include "dijkstra.h"



std::vector<int> Dijkstra::getShortestPath(void)
{
	return shortest_path;
}

 
// Function that implements Dijkstra's single source shortest path algorithm
// for a graph represented using adjacency matrix representation
bool Dijkstra::calcPath(std::vector<std::vector<int>> graph)
{	
	int start = 0; // by definition first element of nodes and therefore also of graph
	int goal = 1; // by definition second element of nodes and therefore also of graph
	
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
		
		if(DIJKSTRA_VERBOSE) {
			std::cout << "u=" << u <<  ", graph.size()=" << graph.size() << std::endl;
		}

		// verify if node u is reachable
		if(dist[u] == DISTANCE_MAX) { continue; }
		
		// Update dist value of the adjacent vertices of the picked vertex.		
		for (int v = 0; v < graph.size(); v++) {
			// verify if node v was not already processed
			if(sptSet[v]) { continue; }
		
			// calc. distance from node u to node v
			int dist_uv;
			if(u==v) { // node v is same than node u
				continue;
			} else if(u>v) { // bottom left triangle
				dist_uv = graph[u][v];
			}else { // top right triangle: mirrow from bottom left triangle
				dist_uv = graph[v][u];
			}
			
			// verify if there is a connection between node u and v
			if(dist_uv<=0) { continue; }
			
			
			// Update dist[v] only if total weight of path from src to  v through u is
			// smaller than current value of dist[v]
			if (dist[u] + dist_uv < dist[v]) {
				dist[v] = dist[u] + dist_uv;
				prev[v] = u;
				
				if(DIJKSTRA_VERBOSE) {
					std::cout << "---(u=" << u << ",v=" << v <<"), dist_uv=" << dist_uv << " sptSet=" 
								 << sptSet[v] << " dist[u]=" << dist[u] << " dist[v]=" << dist[v] 
								 << " prev[v]=" << prev[v] << std::endl;
				}
			}
		}
	}
	
	// calc. shortest path
	shortest_path.clear();
	
	shortest_path.push_back(start);	
	int current = start;
	
	while(current != goal) {
		if(DIJKSTRA_VERBOSE) {
			std::cout << "current=" << current << ", prev=" << prev[current] << std::endl;
		}
		
		shortest_path.push_back(prev[current]);
		current = prev[current];
		
		if(current<0) { return false; } // no path was found
	}
	return true;
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
