#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <utility>
#include <unordered_map>

#include "road.hpp"

// cars
struct Car {
    int id;
    int startNode;
    int endNode;
    std::vector<int> path;
    int pathIndex = 0;
    float progress = 0.f;
    float speed = 120.f;
    sf::Color color;
    sf::Vector2f position;
    bool onEdge = false;
    std::pair<int, int> reservedEdge = { -1,-1 };

    int currentNode() const { return pathIndex; }
    int nextNode() const { return (pathIndex + 1 < (int)path.size()) ? path[pathIndex + 1] : -1; }
};

// initialize a set of cars
std::vector<Car> initCars(int numCars,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    int minspeed, int maxspeed);

// update all cars for a simulation tick
void updateCars(std::vector<Car>& cars,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    std::unordered_map<long long, std::vector<std::pair<int, float>>>& edgeOccupants,
    float dt,
    float minSpacing,
    float ROAD_THICKNESS);
