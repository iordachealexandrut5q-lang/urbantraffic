#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <utility>

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
