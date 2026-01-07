#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <utility>
#include <unordered_map>

#include "road.hpp"
#include "edgeoccupant.hpp"

// EdgeOccupant represents a vehicle occupying an edge (directed road segment).
// Defined in its own header `edgeoccupant.hpp`.

// cars
class Car {
public:
    int id;
    int startNode;
    int endNode;
    int originalEnd = -1; // remember original destination when diverting to POI
    std::vector<int> path;
    int pathIndex = 0;
    float progress = 0.f;
    float speed = 120.f; // desired cruise speed (pixels/sec)
    float velocity = 0.f; // current speed (pixels/sec) - for smooth acceleration
    float accel = 40.f;   // max acceleration (pixels/sec^2)
    float braking = 120.f; // max deceleration (pixels/sec^2)
    sf::Color color;
    sf::Vector2f position;
    bool onEdge = false;
    std::pair<int, int> reservedEdge = { -1,-1 };
    int lane = 0; // current lane on the edge (0..3)

    bool divertedToPOI = false; // whether currently heading to POI instead of original end

    int currentNode() const { return pathIndex; }
    int nextNode() const { return (pathIndex + 1 < (int)path.size()) ? path[pathIndex + 1] : -1; }

    // initialize a set of cars
    static std::vector<Car> initCars(int numCars,
        const std::vector<std::vector<Road>>& graph,
        const std::vector<Intersection>& positions,
        int minspeed, int maxspeed);

    // update all cars for a simulation tick
    // update all cars for a simulation tick (original name)
    static void update(std::vector<Car>& cars,
        const std::vector<std::vector<Road>>& graph,
        const std::vector<Intersection>& positions,
        std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
        float dt,
        float minSpacing,
        float ROAD_THICKNESS,
        const std::vector<int>& pois,
        float poichance);

    // alternate name to improve clarity and match other code variants
    static void updateCars(std::vector<Car>& cars,
        const std::vector<std::vector<Road>>& graph,
        const std::vector<Intersection>& positions,
        std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
        float dt,
        float minSpacing,
        float ROAD_THICKNESS,
        const std::vector<int>& pois,
        float poichance);

private:
    static float roadLength(const std::vector<Intersection>& positions, int a, int b);
};

// buses - derive from Car
class Bus : public Car {
public:
    std::vector<int> stops; // sequence of stops the bus must visit
    int stopIndex = 0; // current target stop index in `stops`

    // initialize buses; startId is used so bus ids don't collide with car ids
    static std::vector<Bus> initBuses(int numBuses,
        const std::vector<std::vector<Road>>& graph,
        const std::vector<Intersection>& positions,
        int minspeed, int maxspeed,
        int startId = 0,
        int stopsPerBus = 3);

    // update buses each tick (similar to Car::updateCars but advances through multiple stops)
    // update buses each tick (similar to Car::update but advances through multiple stops)
    static void update(std::vector<Bus>& buses,
        const std::vector<std::vector<Road>>& graph,
        const std::vector<Intersection>& positions,
        std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
        float dt,
        float minSpacing,
        float ROAD_THICKNESS);

    // alternate name to improve clarity and match other code variants
    static void updateBuses(std::vector<Bus>& buses,
        const std::vector<std::vector<Road>>& graph,
        const std::vector<Intersection>& positions,
        std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
        float dt,
        float minSpacing,
        float ROAD_THICKNESS);
};
