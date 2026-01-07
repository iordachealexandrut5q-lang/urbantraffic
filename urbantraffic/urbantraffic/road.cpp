#include "road.hpp"

// remove a road (both directions)
void removeRoad(std::vector<std::vector<Road>>& graph, int i, int j) {
    for (auto& r : graph[i]) if (r.destination == j) r.active = false;
    for (auto& r : graph[j]) if (r.destination == i) r.active = false;
}

// remove one direction of the road, creating a one-way road
//to-do
void oneway(std::vector<std::vector<Road>>& graph, int i, int j) {};
