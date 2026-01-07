// roads and intersections
#pragma once
#include <vector>

class Road {
public:
    int destination;
    double distance;
    bool active = true;
    int limit; // speed limit currently unused

    // remove a road (both directions)
    static void removeRoad(std::vector<std::vector<Road>>& graph, int i, int j);

    // remove one direction of the road, creating a one-way road
    static void oneway(std::vector<std::vector<Road>>& graph, int i, int j);
};

class Intersection {
public:
    float x, y;
};
