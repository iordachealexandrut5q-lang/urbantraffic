#include "car.hpp"
#include "utils.hpp"
#include "pathfinding.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

extern std::mt19937 rng; // from utils.cpp

std::vector<Car> initCars(int numCars,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    int minspeed, int maxspeed) {
    int numNodes = (int)positions.size();
    std::vector<Car> cars;
    cars.reserve(numCars);
    for (int i = 0; i < numCars; ++i) {
        int s = randint(0, numNodes - 1);
        int e = s;
        while (e == s) e = randint(0, numNodes - 1);
        Car car;
        car.id = i;
        car.startNode = s; //node at which the car spawns
        car.endNode = e; //destination node
        car.color = sf::Color((sf::Uint8)randint(50, 255), (sf::Uint8)randint(50, 255), (sf::Uint8)randint(50, 255));
        car.speed = randfloat((float)minspeed, (float)maxspeed); // used for the variation of speeds
        car.path = dijkstra(graph, car.startNode, car.endNode);
        if (car.path.empty()) {
            car.path = { car.startNode };
        }
        car.pathIndex = 0;
        car.progress = 0.f;
        car.onEdge = false;
        car.reservedEdge = { -1,-1 };
        car.position = { positions[car.startNode].x, positions[car.startNode].y };
        cars.push_back(car);
    }
    return cars;
}

// helper for edge length
static float edgeLength(const std::vector<Intersection>& positions, int a, int b) {
    sf::Vector2f pa(positions[a].x, positions[a].y), pb(positions[b].x, positions[b].y);
    return std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
}

void updateCars(std::vector<Car>& cars,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    std::unordered_map<long long, std::vector<std::pair<int, float>>>& edgeOccupants,
    float dt,
    float minSpacing,
    float ROAD_THICKNESS) {

    int numNodes = (int)positions.size();

    // shuffled order
    std::vector<int> order(cars.size());
    for (int i = 0; i < (int)cars.size(); ++i) order[i] = i;
    std::shuffle(order.begin(), order.end(), rng);

    for (int idx : order) {
        Car& car = cars[idx];

        // if path length is 0 or pathIndex beyond, recompute path
        if (car.path.empty()) {
            car.path = dijkstra(graph, car.pathIndex, car.endNode);
            car.pathIndex = 0;
            car.onEdge = false;
            car.reservedEdge = { -1,-1 };
        }

        // if car has reached destination (pathIndex == path.size()-1)
        if (car.pathIndex >= (int)car.path.size() - 1) {
            // reached end
            if (car.path.size() >= 1 && car.path.back() == car.endNode) {
                int oldStart = car.startNode;
                car.startNode = car.endNode;
                car.endNode = oldStart;
                car.path = dijkstra(graph, car.path.back(), car.endNode);
                car.pathIndex = 0;
                car.progress = 0.f;
                car.onEdge = false;
                car.reservedEdge = { -1,-1 };
                if (car.path.empty()) {
                    car.path = { car.startNode };
                }
            }
            else {
                car.path = dijkstra(graph, car.pathIndex, car.endNode);
                if (car.path.empty()) {
                    int r = randint(0, numNodes - 1);
                    car.endNode = r == car.startNode ? (r + 1) % numNodes : r;
                    car.path = dijkstra(graph, car.startNode, car.endNode);
                }
            }
            continue;
        }

        // if not on edge, attempt to enter directed edge cur -> nxt
        if (!car.onEdge) {
            int cur = car.path[car.pathIndex];
            int nxt = car.nextNode();
            if (nxt == -1) continue;

            // check edge still active; if not, recompute path from current node
            bool edgeActive = false;
            for (auto& r : graph[cur]) if (r.destination == nxt && r.active) { edgeActive = true; break; }
            if (!edgeActive) {
                std::vector<int> newPath = dijkstra(graph, cur, car.endNode);
                if (newPath.empty()) {
                    // wait
                    continue;
                }
                else {
                    car.path = newPath;
                    car.pathIndex = 0;
                    car.onEdge = false;
                    car.reservedEdge = { -1,-1 };
                    continue;
                }
            }

            // check directed occupancy for cur->nxt
            long long k = edgeKey(cur, nxt);
            float segLen = edgeLength(positions, cur, nxt);

            bool canEnter = false;
            auto& vec = edgeOccupants[k];
            if (vec.empty()) {
                canEnter = true;
            }
            else {
                // find front-most car progress (max progress)
                float frontProg = 0.f;
                for (auto& p : vec) frontProg = std::max(frontProg, p.second);
                float frontDist = frontProg * segLen;
                if (frontDist >= minSpacing + 1.0f) canEnter = true;
                else canEnter = false;
            }

            if (canEnter) {
                // add occupant at progress 0
                vec.push_back({ car.id, 0.f });
                car.onEdge = true;
                car.reservedEdge = { cur, nxt };
                car.progress = 0.f;
                car.position = { positions[cur].x, positions[cur].y };
            }
            else {
                // wait at node
                continue;
            }
        } // end try-enter

        // if on edge, move along it but maintain spacing to car ahead (if any)
        if (car.onEdge) {
            int a = car.reservedEdge.first;
            int b = car.reservedEdge.second;
            if (a == -1 || b == -1) { car.onEdge = false; continue; }
            sf::Vector2f pa(positions[a].x, positions[a].y), pb(positions[b].x, positions[b].y);
            float segLen = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
            if (segLen <= 0.001f) {
                car.position = pb;
                // remove occupant entry
                long long k = edgeKey(a, b);
                auto& vec = edgeOccupants[k];
                vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const std::pair<int, float>& p) { return p.first == car.id; }), vec.end());
                if (vec.empty()) edgeOccupants.erase(k);
                car.onEdge = false;
                car.reservedEdge = { -1,-1 };
                car.pathIndex++;
                car.progress = 0.f;
                continue;
            }

            // find this car's record in edgeOccupants[a->b]
            long long k = edgeKey(a, b);
            auto& vec = edgeOccupants[k];
            auto it = std::find_if(vec.begin(), vec.end(), [&](const std::pair<int, float>& p) { return p.first == car.id; });
            if (it == vec.end()) {
                // should not happen; reinsert
                vec.push_back({ car.id, car.progress });
                it = std::find_if(vec.begin(), vec.end(), [&](const std::pair<int, float>& p) { return p.first == car.id; });
            }

            // find the smallest progress strictly greater than this car's progress (i.e., car ahead)
            float myProg = it->second;
            float nextAheadProg = 2.0f; // >1 means none
            for (auto& p : vec) {
                if (p.first == car.id) continue;
                if (p.second > myProg) nextAheadProg = std::min(nextAheadProg, p.second);
            }

            // compute allowed maximum progress considering spacing
            float allowedMaxProg = 1.0f;
            if (nextAheadProg <= 1.0f) {
                float allowedDist = nextAheadProg * segLen - minSpacing;
                allowedMaxProg = std::max(0.0f, allowedDist / segLen);
            }

            // compute attempted step
            float step = (car.speed * dt) / segLen;
            float targetProg = myProg + step;
            if (targetProg > allowedMaxProg) targetProg = allowedMaxProg;

            // update progress and position
            it->second = targetProg;
            car.progress = targetProg;
            if (car.progress >= 1.0f - 1e-5f) {
                // arrive
                car.position = pb;
                // remove occupant entry
                vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const std::pair<int, float>& p) { return p.first == car.id; }), vec.end());
                if (vec.empty()) edgeOccupants.erase(k);
                car.onEdge = false;
                car.reservedEdge = { -1,-1 };
                car.pathIndex++;
                car.progress = 0.f;
            }
            else {
                // compute position along segment
                sf::Vector2f dir = pb - pa;
                float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                if (len > 1e-5f) dir /= len;

                // perpendicular vector to the right side
                sf::Vector2f right(-dir.y, dir.x);

                // lane offset - will need to be tweaked for 4 lane roads
                float laneOffset = ROAD_THICKNESS * 0.3f;

                // apply offset to move car to the right lane
                sf::Vector2f pos = pa + dir * (car.progress * len) + right * laneOffset;
                car.position = pos;

            }
        } // end onEdge handling
    }
}