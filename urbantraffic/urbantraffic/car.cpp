#include "car.hpp"
#include "utils.hpp"
#include "pathfinding.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

std::vector<Car> initCars(const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    int numCars) {
    std::vector<Car> cars;
    cars.reserve(numCars);
    int numNodes = positions.size();

    for (int i = 0; i < numCars; ++i) {
        int s = randint(0, numNodes - 1);
        int e = s;
        while (e == s) e = randint(0, numNodes - 1);

        Car car;
        car.id = i;
        car.startNode = s;
        car.endNode = e;
        car.color = sf::Color((sf::Uint8)randint(50, 255),
            (sf::Uint8)randint(50, 255),
            (sf::Uint8)randint(50, 255));
        car.speed = randfloat(80.f, 150.f);
        car.path = dijkstra(graph, car.startNode, car.endNode);
        if (car.path.empty()) car.path = { car.startNode };
        car.pathIndex = 0;
        car.progress = 0.f;
        car.position = { positions[s].x, positions[s].y };
        car.onEdge = false;
        car.reservedEdge = { -1,-1 };
        cars.push_back(car);
    }
    return cars;
}
void updateCars(std::vector<Car>& cars,
    std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    float dt,
    std::unordered_map<long long, std::vector<std::pair<int, float>>>& edgeOccupants,
    float minSpacing,
    float roadThickness) {
    auto edgeLength = [&](int a, int b) {
        sf::Vector2f pa(positions[a].x, positions[a].y), pb(positions[b].x, positions[b].y);
        return sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
        };

    std::vector<int> order(cars.size());
    for (int i = 0; i < (int)cars.size(); ++i) order[i] = i;
    shuffle(order.begin(), order.end(), rng);

    for (int idx : order) {
        Car& car = cars[idx];
        if (car.path.empty()) continue;

        if (car.pathIndex >= (int)car.path.size() - 1) {
            int oldStart = car.startNode;
            car.startNode = car.endNode;
            car.endNode = oldStart;
            car.path = dijkstra(graph, car.startNode, car.endNode);
            car.pathIndex = 0;
            car.progress = 0.f;
            car.onEdge = false;
            car.reservedEdge = { -1,-1 };
            continue;
        }

        int cur = car.path[car.pathIndex];
        int nxt = car.nextNode();
        if (nxt == -1) continue;

        long long k = edgeKey(cur, nxt);
        float segLen = edgeLength(cur, nxt);
        auto& vec = edgeOccupants[k];

        if (!car.onEdge) {
            bool edgeActive = false;
            for (auto& r : graph[cur])
                if (r.destination == nxt && r.active) edgeActive = true;
            if (!edgeActive) continue;

            bool canEnter = true;
            if (!vec.empty()) {
                float frontProg = 0.f;
                for (auto& p : vec) frontProg = std::max(frontProg, p.second);
                float frontDist = frontProg * segLen;
                canEnter = (frontDist >= minSpacing + 1.f);
            }

            if (canEnter) {
                vec.push_back({ car.id, 0.f });
                car.onEdge = true;
                car.reservedEdge = { cur, nxt };
                car.progress = 0.f;
            }
            else continue;
        }

        // Move along the segment
        float step = (car.speed * dt) / segLen;
        auto it = std::find_if(vec.begin(), vec.end(),
            [&](auto& p) { return p.first == car.id; });
        if (it == vec.end()) continue;

        float myProg = it->second + step;
        it->second = myProg;
        car.progress = myProg;

        sf::Vector2f pa(positions[cur].x, positions[cur].y), pb(positions[nxt].x, positions[nxt].y);
        sf::Vector2f dir = pb - pa;
        float len = sqrt(dir.x * dir.x + dir.y * dir.y);
        if (len > 1e-5f) dir /= len;
        sf::Vector2f right(-dir.y, dir.x);
        car.position = pa + dir * (car.progress * len) + right * (roadThickness * 0.3f);

        if (car.progress >= 1.f) {
            car.pathIndex++;
            car.onEdge = false;
            car.reservedEdge = { -1,-1 };
            vec.erase(remove_if(vec.begin(), vec.end(),
                [&](auto& p) { return p.first == car.id; }),
                vec.end());
        }
    }
}
