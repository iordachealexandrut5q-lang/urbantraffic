// roads and intersections
#pragma once
#include <vector>

struct Road {
    int destination;
    double distance;
    bool active = true;
    int limit; // speed limit currently unused
};

struct Intersection {
    float x, y;
};

// remove a road (both directions)
void removeRoad(std::vector<std::vector<Road>>& graph, int i, int j);

// remove one direction of the road, creating a one-way road
//to-do
void oneway(std::vector<std::vector<Road>>& graph, int i, int j);
