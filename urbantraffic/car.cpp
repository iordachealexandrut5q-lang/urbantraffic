#include "car.hpp"
#include "utils.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>

std::vector<Car> Car::initCars(int numCars,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    int minspeed, int maxspeed,
    bool enableCommute) {
    int numNodes = (int)positions.size();
    std::vector<Car> cars;
    cars.reserve(numCars);
    for (int i = 0; i < numCars; ++i) {
        int s = Utils::randint(0, numNodes - 1);
        int e = s;
        while (e == s) e = Utils::randint(0, numNodes - 1);
        Car car;
        car.id = i;
        car.startNode = s; //node at which the car spawns
        car.endNode = e; //destination node
        car.originalEnd = e;
        car.color = sf::Color((sf::Uint8)Utils::randint(50, 255), (sf::Uint8)Utils::randint(50, 255), (sf::Uint8)Utils::randint(50, 255));
        car.speed = Utils::randfloat((float)minspeed, (float)maxspeed); // used for the variation of speeds (desired)
        car.velocity = car.speed * 0.5f; // start slower than desired
        car.path = Utils::dijkstra(graph, car.startNode, car.endNode);
        if (car.path.empty()) {
            car.path = { car.startNode };
        }
        car.pathIndex = 0;
        car.progress = 0.f;
        car.onEdge = false;
        car.reservedEdge = { -1,-1 };
        car.position = { positions[car.startNode].x, positions[car.startNode].y };
        // choose a random lane among the two rightmost lanes only (lanes 0 and 1)
        car.lane = Utils::randint(0, 1);

        // commuter assignment: ~80% become commuters with home/work if enabled
        if (enableCommute) {
            float r = Utils::randfloat(0.f, 1.f);
            if (r <= 0.8f) {
                car.isCommuter = true;
                car.homeNode = car.startNode;
				// pick a work node different from home
                int w = car.homeNode;
                while (w == car.homeNode) w = Utils::randint(0, numNodes - 1);
                car.workNode = w;
                // start day at home
                car.commuteState = Car::AT_HOME;
				// set destination to work for when commute starts
                car.endNode = car.workNode;
                car.originalEnd = car.endNode;
                car.path = Utils::dijkstra(graph, car.startNode, car.endNode);
                if (car.path.empty()) car.path = { car.startNode };
            }
        }

        cars.push_back(car);
    }
    return cars;
}

// helper for road length
float Car::roadLength(const std::vector<Intersection>& positions, int a, int b) {
    sf::Vector2f pa(positions[a].x, positions[a].y), pb(positions[b].x, positions[b].y);
    return std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
}

void Car::update(std::vector<Car>& cars,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
    float dt,
    float minSpacing,
    float ROAD_THICKNESS,
    const std::vector<int>& pois,
    float poichance,
    bool commuteEnabled,
    float simTime) {

    const int NUM_LANES = 4;
    const int CAR_MIN_LANE = 0; // rightmost lanes are 0 and 1
    const int CAR_MAX_LANE = 1;

    int numNodes = (int)positions.size();

    // build id -> index map to avoid linear scans when looking up cars by id
    std::unordered_map<int, int> idToIndex;
    idToIndex.reserve(cars.size() * 2 + 1);
    for (int i = 0; i < (int)cars.size(); ++i) idToIndex[cars[i].id] = i;

    // shuffled order
    std::vector<int> order(cars.size());
    for (int i = 0; i < (int)cars.size(); ++i) order[i] = i;
    std::shuffle(order.begin(), order.end(), Utils::rng);

    // define commute schedule hours
    const float DEPART_HOUR = 6.0f; // 6:00 AM
    const float RETURN_HOUR = 17.0f; // 5:00 PM

    for (int idx : order) {
        Car& car = cars[idx];

        // handle commuter schedule state transitions if commute feature enabled
        if (commuteEnabled && car.isCommuter) {
            // if at home and time is depart hour or later (but before return), start heading to work
            if (car.commuteState == Car::AT_HOME) {
                if (simTime >= DEPART_HOUR && simTime < RETURN_HOUR) {
                    // start trip to work
                    car.endNode = car.workNode;
                    car.path = Utils::dijkstra(graph, car.startNode, car.endNode);
                    car.pathIndex = 0;
                    car.onEdge = false;
                    car.reservedEdge = { -1, -1 };
                    car.progress = 0.f;
                    car.commuteState = Car::TO_WORK;
                    if (car.path.empty()) { car.path = { car.startNode }; }
                }
            }
            // if at work and time is return hour or later, start heading home
            else if (car.commuteState == Car::AT_WORK) {
                if (simTime >= RETURN_HOUR || simTime < DEPART_HOUR) { // allow wrap-around midnight
                    car.endNode = car.homeNode;
                    car.path = Utils::dijkstra(graph, car.startNode, car.endNode);
                    car.pathIndex = 0;
                    car.onEdge = false;
                    car.reservedEdge = { -1, -1 };
                    car.progress = 0.f;
                    car.commuteState = Car::TO_HOME;
                    if (car.path.empty()) { car.path = { car.startNode }; }
                }
            }
        }

        // if path length is 0 or pathIndex beyond, recompute path
        if (car.path.empty()) {
            car.path = Utils::dijkstra(graph, car.pathIndex, car.endNode);
            car.pathIndex = 0;
            car.onEdge = false;
            car.reservedEdge = { -1,-1 };
        }

        // if car has reached destination (pathIndex == path.size()-1)
        if (car.pathIndex >= (int)car.path.size() - 1) {
            // reached end
            if (car.path.size() >= 1 && car.path.back() == car.endNode) {
                // For commuters, when they reach their destination update state and enforce waiting until schedule
                if (commuteEnabled && car.isCommuter) {
                    if (car.commuteState == Car::TO_WORK) {
                        // arrived at work - switch to AT_WORK and stay until RETURN_HOUR
                        car.startNode = car.endNode;
                        car.path = { car.startNode };
                        car.pathIndex = 0;
                        car.progress = 0.f;
                        car.onEdge = false;
                        car.reservedEdge = { -1,-1 };
                        car.commuteState = Car::AT_WORK;
                        // set position to node
                        car.position = { positions[car.startNode].x, positions[car.startNode].y };
                        car.velocity = 0.f;
                        continue; // skip further movement
                    }
                    else if (car.commuteState == Car::TO_HOME) {
                        // arrived at home - switch to AT_HOME and stay until next day DEPART_HOUR
                        car.startNode = car.endNode;
                        car.path = { car.startNode };
                        car.pathIndex = 0;
                        car.progress = 0.f;
                        car.onEdge = false;
                        car.reservedEdge = { -1,-1 };
                        car.commuteState = Car::AT_HOME;
                        car.position = { positions[car.startNode].x, positions[car.startNode].y };
                        car.velocity = 0.f;
                        continue;
                    }
                }

                // Non-commuter or normal behavior: existing POI diversion logic and swap start/end
                // before swapping start/end, check POI diversion chance
                if (!pois.empty() && !car.divertedToPOI) {
                    float r = Utils::randfloat(0.f, 1.f);
                    if (r < poichance) {
                        // pick a random POI different from current end
                        int poi = pois[Utils::randint(0, (int)pois.size() - 1)];
                        if (poi != car.endNode) {
                            car.originalEnd = car.endNode;
                            car.endNode = poi;
                            car.path = Utils::dijkstra(graph, car.path.back(), car.endNode);
                            car.pathIndex = 0;
                            car.progress = 0.f;
                            car.onEdge = false;
                            car.reservedEdge = { -1,-1 };
                            car.divertedToPOI = true;
                            if (car.path.empty()) car.path = { car.startNode };
                            continue;
                        }
                    }
                }

                int oldStart = car.startNode;
                car.startNode = car.endNode;
                car.endNode = oldStart;
				// if was diverted to POI, restore original end
                if (car.divertedToPOI) {
                    car.divertedToPOI = false;
                    car.endNode = car.originalEnd;
                }

                car.path = Utils::dijkstra(graph, car.path.back(), car.endNode);
                car.pathIndex = 0;
                car.progress = 0.f;
                car.onEdge = false;
                car.reservedEdge = { -1,-1 };
                if (car.path.empty()) {
                    car.path = { car.startNode };
                }
            }
            else {
                car.path = Utils::dijkstra(graph, car.pathIndex, car.endNode);
                if (car.path.empty()) {
                    int r = Utils::randint(0, numNodes - 1);
                    car.endNode = r == car.startNode ? (r + 1) % numNodes : r;
                    car.path = Utils::dijkstra(graph, car.startNode, car.endNode);
                }
            }
            continue;
        }

		// handle commuter waiting at home/work
        if (commuteEnabled && car.isCommuter) {
            if (car.commuteState == Car::AT_HOME) {
                if (!(simTime >= DEPART_HOUR && simTime < RETURN_HOUR)) {
					// if it's not yet depart hour, stay at home
                    car.velocity = 0.f;
                    car.position = { positions[car.startNode].x, positions[car.startNode].y };
                    continue;
                }
            }
            if (car.commuteState == Car::AT_WORK) {
                // if it's not yet return hour, stay at work
                if (!(simTime >= RETURN_HOUR || simTime < DEPART_HOUR)) {
                    car.velocity = 0.f;
                    car.position = { positions[car.startNode].x, positions[car.startNode].y };
                    continue;
                }
            }
        }

        // if not on edge, attempt to enter directed edge cur -> nxt
        if (!car.onEdge) {
            int cur = car.path[car.pathIndex];
            int nxt = car.nextNode();
            if (nxt == -1) continue;

			// check if edge cur->nxt is active
            bool edgeActive = false;
            for (auto& r : graph[cur]) if (r.destination == nxt && r.active) { edgeActive = true; break; }
            if (!edgeActive) {
                std::vector<int> newPath = Utils::dijkstra(graph, cur, car.endNode);
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
            long long k = Utils::edgeKey(cur, nxt);
            float segLen = roadLength(positions, cur, nxt);

            auto& vec = edgeOccupants[k];

            // attempt to find a lane with sufficient space near start
            int chosenLane = -1;
            float requiredDist = minSpacing + 5.0f; // small buffer for entering
			// check lanes from rightmost to leftmost
            for (int lane = CAR_MIN_LANE; lane <= CAR_MAX_LANE; ++lane) {
                float frontProg = -1.f; // -1 means empty
                for (auto& p : vec) {
                    if (p.lane != lane) continue;
                    frontProg = std::max(frontProg, p.progress);
                }
                if (frontProg < 0.f) {
                    // lane empty -> can enter
                    chosenLane = lane;
                    break;
                }
                else {
                    float frontDist = frontProg * segLen;
                    if (frontDist >= requiredDist) { chosenLane = lane; break; }
                }
            }

            if (chosenLane >= 0) {
                EdgeOccupant occ; occ.id = car.id; occ.progress = 0.f; occ.lane = chosenLane;
                vec.push_back(occ);
                car.onEdge = true;
                car.reservedEdge = { cur, nxt };
                car.progress = 0.f;
                car.position = { positions[cur].x, positions[cur].y };
                car.lane = chosenLane;
            }
            else {
                // wait at node (blocked) - car decelerates to 0 smoothly
                if (car.velocity > 0.f) {
                    car.velocity -= car.braking * dt;
                    if (car.velocity < 0.f) car.velocity = 0.f;
                }
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
				// remove from edgeOccupants
                long long k = Utils::edgeKey(a, b);
                auto& vec = edgeOccupants[k]; 
                vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == car.id; }), vec.end()); 
				if (vec.empty()) edgeOccupants.erase(k); // clean up if empty
                car.onEdge = false; 
                car.reservedEdge = { -1,-1 }; 
				car.pathIndex++; // advance to next node
                car.progress = 0.f;
                continue;
            }

            // find this car's record in edgeOccupants[a->b]
            long long k = Utils::edgeKey(a, b);
            auto& vec = edgeOccupants[k];
            auto it = std::find_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == car.id; });
            if (it == vec.end()) {
				// should not happen, but if so, re-add
                vec.push_back({ car.id, car.progress, car.lane });
                it = std::find_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == car.id; });
            }

            // find the car ahead in the same lane
            float myProg = it->progress;
            float nextAheadProg = 2.0f; // >1 means none
            int aheadId = -1;
            for (auto& p : vec) {
                if (p.id == car.id) continue;
                if (p.lane != it->lane) continue; // only consider same lane
				if (p.progress > myProg) { // ahead of car
                    if (p.progress < nextAheadProg) { 
                        nextAheadProg = p.progress; 
                        aheadId = p.id;
                    }
                }
            }

            // lane change logic: if blocked by slower car ahead and adjacent lane offers more gap, change lane
			// note: only attempt lane change if there is a car ahead
            if (nextAheadProg <= 1.0f) {
                // estimate leader velocity using idToIndex map
                float v_lead = 0.f;
                if (aheadId != -1) {
                    auto itidx = idToIndex.find(aheadId);
                    if (itidx != idToIndex.end()) v_lead = cars[itidx->second].velocity;
                }
                // if leader much slower than me and I can change lane, attempt it
                if (v_lead + 5.0f < car.velocity) {
                    // try moving to lane with more space: prefer lane 0 (far-right), then lane 1
                    const int tryDirs[2] = {-1, 1}; // check right first (d=-1), then left (d=1)
                    for (int t = 0; t < 2; ++t) {
                        int d = tryDirs[t];
                        int targetLane = car.lane + d;
						// check lane bounds
                        if (targetLane < CAR_MIN_LANE || targetLane > CAR_MAX_LANE) continue;
                        // check adjacent lane occupancy near our position to ensure safe gap
                        bool safe = true;
                        for (auto& p : vec) {
                            if (p.lane != targetLane) continue;
                            float distAlong = p.progress * segLen;
                            float myDist = myProg * segLen;
                            if (std::abs(distAlong - myDist) < minSpacing * 0.8f) { safe = false; break; }
                        }
                        if (safe) {
                            it->lane = targetLane;
                            car.lane = targetLane;
                            break;
                        }
                    }
                }
            }

            // recompute nextAheadProg for same lane after possible lane change
            myProg = it->progress;
            nextAheadProg = 2.0f; aheadId = -1;
            for (auto& p : vec) {
                if (p.id == car.id) continue;
                if (p.lane != it->lane) continue;
                if (p.progress > myProg) {
                    if (p.progress < nextAheadProg) {
                        nextAheadProg = p.progress;
                        aheadId = p.id;
                    }
                }
            }

			// compute desired gap to car ahead
            float t_headway = 1.0f; 
            float v = car.velocity;
			float v_lead = v; // assume same speed if no leader
            float gap = minSpacing;
            if (nextAheadProg <= 1.0f) {
                v_lead = 0.f; // fallback
                if (aheadId != -1) {
                    auto itidx = idToIndex.find(aheadId);
                    if (itidx != idToIndex.end()) v_lead = cars[itidx->second].velocity;
                }
                gap = minSpacing + v * t_headway + (v * (v - v_lead)) / (2.0f * std::sqrt(car.accel * car.braking));
                if (gap < minSpacing) gap = minSpacing;
            }

            // compute allowed maximum progress considering spacing
            float allowedMaxProg = 1.0f;
            if (nextAheadProg <= 1.0f) {
                float allowedDist = nextAheadProg * segLen - gap;
                allowedMaxProg = std::max(0.0f, allowedDist / segLen);
            }

			// compute desired velocity
            float desiredVel = car.speed;
            float distToAllowedEnd = (allowedMaxProg - myProg) * segLen;
            if (distToAllowedEnd < 0) distToAllowedEnd = 0;

            if (distToAllowedEnd < 1.0f) {
                desiredVel = 0.f;
            }

			// accelerate or decelerate towards desiredVel
            if (car.velocity < desiredVel) {
                car.velocity += car.accel * dt;
                if (car.velocity > desiredVel) car.velocity = desiredVel;
            } else if (car.velocity > desiredVel) {
                car.velocity -= car.braking * dt;
                if (car.velocity < desiredVel) car.velocity = desiredVel;
            }

			// compute step along edge
            float step = (car.velocity * dt) / segLen;
            float targetProg = myProg + step;
            if (targetProg > allowedMaxProg) {
				// apply braking to avoid overshoot
                targetProg = allowedMaxProg;
                float remainDist = (allowedMaxProg - myProg) * segLen;
				// braking
                float desiredV2 = std::max(0.0f, remainDist / std::max(0.0001f, t_headway));
                if (car.velocity > desiredV2) {
                    car.velocity -= car.braking * dt;
                    if (car.velocity < desiredV2) car.velocity = desiredV2;
                }
            }

            // update progress and position
            it->progress = targetProg;
            car.progress = targetProg;
            if (car.progress >= 1.0f - 1e-5f) {
                car.position = pb;
				// remove occupant entry
                vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == car.id; }), vec.end());
                if (vec.empty()) edgeOccupants.erase(k);
                car.onEdge = false;
                car.reservedEdge = { -1,-1 };
                car.pathIndex++;
                car.progress = 0.f;
                // preserve velocity
            }
            else {
				// compute position along edge with lane offset
                sf::Vector2f dir = pb - pa;
                float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                if (len > 1e-5f) dir /= len;

                sf::Vector2f right(-dir.y, dir.x);

                // compute lane offset for 4 lanes: lanes centered around road center
                float laneWidth = ROAD_THICKNESS / (float)NUM_LANES;
                // lanes indices 0..3, lane 0 is far-right, lane 3 far-left
                float centerOffset = (-(NUM_LANES - 1) / 2.0f) * laneWidth; // offset of lane 0 from center
                float laneOffset = centerOffset + car.lane * laneWidth;

                // apply offset to move car to its lane (right is positive)
                sf::Vector2f pos = pa + dir * (car.progress * len) + right * laneOffset;
                car.position = pos;

            }
        } // end onEdge handling
    }
}

std::vector<Bus> Bus::initBuses(int numBuses,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    int minspeed, int maxspeed,
    int startId,
    int stopsPerBus) {
    int numNodes = (int)positions.size();
    std::vector<Bus> buses;
    buses.reserve(numBuses);
    for (int i = 0; i < numBuses; ++i) {
        int s = Utils::randint(0, numNodes - 1);
        Bus bus;
        bus.id = startId + i;
        bus.startNode = s;
        bus.speed = Utils::randfloat((float)minspeed, (float)maxspeed);
        bus.velocity = bus.speed * 0.5f;
        bus.color = sf::Color(200, 180, 0);

        // pick stopsPerBus unique stops
        bus.stops.clear();
        for (int k = 0; k < stopsPerBus; ++k) {
            int node = Utils::randint(0, numNodes - 1);
            // avoid immediate duplicate with previous stop
            if (!bus.stops.empty() && node == bus.stops.back()) {
                node = (node + 1) % numNodes;
            }
            bus.stops.push_back(node);
        }

        bus.stopIndex = 0;
        bus.endNode = bus.stops.empty() ? bus.startNode : bus.stops[0];
        bus.path = Utils::dijkstra(graph, bus.startNode, bus.endNode);
        if (bus.path.empty()) bus.path = { bus.startNode };
        bus.pathIndex = 0;
        bus.progress = 0.f;
        bus.onEdge = false;
        bus.reservedEdge = { -1, -1 };
        bus.position = { positions[bus.startNode].x, positions[bus.startNode].y };
        bus.lane = Utils::randint(0, 3);
        buses.push_back(bus);
    }
    return buses;
}

void Bus::update(std::vector<Bus>& buses,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
    float dt,
    float minSpacing,
    float ROAD_THICKNESS) {

    const int NUM_LANES = 4;

    int numNodes = (int)positions.size();
    if (buses.empty()) return;

    // build id -> index map for buses
    std::unordered_map<int, int> idToIndex;
    idToIndex.reserve(buses.size() * 2 + 1);
    for (int i = 0; i < (int)buses.size(); ++i) idToIndex[buses[i].id] = i;

    std::vector<int> order(buses.size());
    for (int i = 0; i < (int)buses.size(); ++i) order[i] = i;
    std::shuffle(order.begin(), order.end(), Utils::rng);

    for (int idx : order) {
        Bus& bus = buses[idx];

        // ensure current endNode matches target stop
        if (!bus.stops.empty()) bus.endNode = bus.stops[bus.stopIndex];

        if (bus.path.empty()) {
            bus.path = Utils::dijkstra(graph, bus.pathIndex, bus.endNode);
            bus.pathIndex = 0;
            bus.onEdge = false;
            bus.reservedEdge = { -1,-1 };
        }

        // if reached final index in path
        if (bus.pathIndex >= (int)bus.path.size() - 1) {
            if (bus.path.size() >= 1 && bus.path.back() == bus.endNode) {
                // reached current stop - advance to next stop
                bus.startNode = bus.endNode;
                if (!bus.stops.empty()) {
                    bus.stopIndex = (bus.stopIndex + 1) % (int)bus.stops.size();
                    bus.endNode = bus.stops[bus.stopIndex];
                }
                bus.path = Utils::dijkstra(graph, bus.startNode, bus.endNode);
                bus.pathIndex = 0;
                bus.progress = 0.f;
                bus.onEdge = false;
                bus.reservedEdge = { -1,-1 };
                if (bus.path.empty()) bus.path = { bus.startNode };
            }
            else {
                bus.path = Utils::dijkstra(graph, bus.pathIndex, bus.endNode);
                if (bus.path.empty()) {
                    int r = Utils::randint(0, numNodes - 1);
                    bus.endNode = r == bus.startNode ? (r + 1) % numNodes : r;
                    bus.path = Utils::dijkstra(graph, bus.startNode, bus.endNode);
                }
            }
            continue;
        }

        // if not on edge, attempt to enter directed edge
        if (!bus.onEdge) {
            int cur = bus.path[bus.pathIndex];
            int nxt = (bus.pathIndex + 1 < (int)bus.path.size()) ? bus.path[bus.pathIndex + 1] : -1;
            if (nxt == -1) continue;

            bool edgeActive = false;
            for (auto& r : graph[cur]) if (r.destination == nxt && r.active) { edgeActive = true; break; }
            if (!edgeActive) {
                std::vector<int> newPath = Utils::dijkstra(graph, cur, bus.endNode);
                if (newPath.empty()) continue;
                bus.path = newPath; bus.pathIndex = 0; bus.onEdge = false; bus.reservedEdge = { -1,-1 }; continue;
            }

            long long k = Utils::edgeKey(cur, nxt);
            sf::Vector2f pa(positions[cur].x, positions[cur].y), pb(positions[nxt].x, positions[nxt].y);
            float segLen = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));

            auto& vec = edgeOccupants[k];

            int chosenLane = -1;
            float requiredDist = minSpacing + 10.0f; // larger buffer for buses
            for (int lane = 0; lane < NUM_LANES; ++lane) {
                float frontProg = -1.f;
                for (auto& p : vec) {
                    if (p.lane != lane) continue;
                    frontProg = std::max(frontProg, p.progress);
                }
                if (frontProg < 0.f) { chosenLane = lane; break; }
                else {
                    float frontDist = frontProg * segLen;
                    if (frontDist >= requiredDist) { chosenLane = lane; break; }
                }
            }

            if (chosenLane >= 0) {
                EdgeOccupant occ; occ.id = bus.id; occ.progress = 0.f; occ.lane = chosenLane;
                vec.push_back(occ);
                bus.onEdge = true;
                bus.reservedEdge = { cur, nxt };
                bus.progress = 0.f;
                bus.position = { positions[cur].x, positions[cur].y };
                bus.lane = chosenLane;
            }
            else continue;
        }

        // on edge movement
        if (bus.onEdge) {
            int a = bus.reservedEdge.first;
            int b = bus.reservedEdge.second;
            if (a == -1 || b == -1) { bus.onEdge = false; continue; }
            sf::Vector2f pa(positions[a].x, positions[a].y), pb(positions[b].x, positions[b].y);
            float segLen = std::sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
            if (segLen <= 0.001f) {
                bus.position = pb;
                long long k = Utils::edgeKey(a, b);
                auto& vec = edgeOccupants[k];
                vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == bus.id; }), vec.end());
                if (vec.empty()) edgeOccupants.erase(k);
                bus.onEdge = false;
                bus.reservedEdge = { -1,-1 };
                bus.pathIndex++;
                bus.progress = 0.f;
                continue;
            }

            long long k = Utils::edgeKey(a, b);
            auto& vec = edgeOccupants[k];
            auto it = std::find_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == bus.id; });
            if (it == vec.end()) { vec.push_back({ bus.id, bus.progress, bus.lane }); it = std::find_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == bus.id; }); }

            float myProg = it->progress;
            float nextAheadProg = 2.0f;
            int aheadId = -1;
            for (auto& p : vec) {
                if (p.id == bus.id) continue;
                if (p.lane != it->lane) continue;
                if (p.progress > myProg) {
                    if (p.progress < nextAheadProg) {
                        nextAheadProg = p.progress;
                        aheadId = p.id;
                    }
                }
            }

            float t_headway = 1.2f; // buses keep slightly larger headway
            float v = bus.velocity;
            float v_lead = v;
            float gap = minSpacing + 10.0f; // buses are larger
            if (nextAheadProg <= 1.0f) {
                v_lead = 0.f;
                if (aheadId != -1) {
                    auto itidx = idToIndex.find(aheadId);
                    if (itidx != idToIndex.end()) v_lead = buses[itidx->second].velocity;
                }
                gap = minSpacing + v * t_headway + (v * (v - v_lead)) / (2.0f * std::sqrt(bus.accel * bus.braking));
                if (gap < minSpacing) gap = minSpacing;
            }

            float allowedMaxProg = 1.0f;
            if (nextAheadProg <= 1.0f) {
                float allowedDist = nextAheadProg * segLen - gap;
                allowedMaxProg = std::max(0.0f, allowedDist / segLen);
            }

            float desiredVel = bus.speed;
            float distToAllowedEnd = (allowedMaxProg - myProg) * segLen;
            if (distToAllowedEnd < 0) distToAllowedEnd = 0;
            if (distToAllowedEnd < 1.0f) desiredVel = 0.f;

            if (bus.velocity < desiredVel) {
                bus.velocity += bus.accel * dt;
                if (bus.velocity > desiredVel) bus.velocity = desiredVel;
            } else if (bus.velocity > desiredVel) {
                bus.velocity -= bus.braking * dt;
                if (bus.velocity < desiredVel) bus.velocity = desiredVel;
            }

            float step = (bus.velocity * dt) / segLen;
            float targetProg = myProg + step;
            if (targetProg > allowedMaxProg) {
                targetProg = allowedMaxProg;
                float remainDist = (allowedMaxProg - myProg) * segLen;
                float desiredV2 = std::max(0.0f, remainDist / std::max(0.0001f, t_headway));
                if (bus.velocity > desiredV2) {
                    bus.velocity -= bus.braking * dt;
                    if (bus.velocity < desiredV2) bus.velocity = desiredV2;
                }
            }

            it->progress = targetProg;
            bus.progress = targetProg;
            if (bus.progress >= 1.0f - 1e-5f) {
                bus.position = pb;
                vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const EdgeOccupant& p) { return p.id == bus.id; }), vec.end());
                if (vec.empty()) edgeOccupants.erase(k);
                bus.onEdge = false;
                bus.reservedEdge = { -1,-1 };
                bus.pathIndex++;
                bus.progress = 0.f;
            }
            else {
                sf::Vector2f dir = pb - pa;
                float len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
                if (len > 1e-5f) dir /= len;
                sf::Vector2f right(-dir.y, dir.x);
                float laneWidth = ROAD_THICKNESS / (float)NUM_LANES;
                float centerOffset = (-(NUM_LANES - 1) / 2.0f) * laneWidth;
                float laneOffset = centerOffset + bus.lane * laneWidth;
                sf::Vector2f pos = pa + dir * (bus.progress * len) + right * laneOffset;
                bus.position = pos;
            }
        }
    }
}

// wrapper to match alternate naming used elsewhere
void Car::updateCars(std::vector<Car>& cars,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
    float dt,
    float minSpacing,
    float ROAD_THICKNESS,
    const std::vector<int>& pois,
    float poichance,
    bool commuteEnabled,
    float simTime) {
    Car::update(cars, graph, positions, edgeOccupants, dt, minSpacing, ROAD_THICKNESS, pois, poichance, commuteEnabled, simTime);
}

// wrapper for buses
void Bus::updateBuses(std::vector<Bus>& buses,
    const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    std::unordered_map<long long, std::vector<EdgeOccupant>>& edgeOccupants,
    float dt,
    float minSpacing,
    float ROAD_THICKNESS) {
    Bus::update(buses, graph, positions, edgeOccupants, dt, minSpacing, ROAD_THICKNESS);
}