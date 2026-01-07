#pragma once
#include <vector>
#include "road.hpp"

// dijkstra algorithm to calculate the optimal path each car must take from their start node to their end node
// following the active roads only and updating in real time with the deletion of roads
std::vector<int> dijkstra(const std::vector<std::vector<Road>>& graph, int start, int goal);
