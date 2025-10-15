//urban traffic simulator v 0.0.1
//read readme

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <limits>
#include <random>
#include <set>
#include <algorithm>
#include <unordered_map>

using namespace std;


// roads and intersections
struct Road {
    int destination;
    double distance;
    bool active = true;
};

struct Intersection {
    float x, y;
};

// generates a graph of nodes connected by roads in a rectangular grid

vector<vector<Road>> generateCityGrid(int rows, int cols) {
    int numNodes = rows * cols;
    vector<vector<Road>> graph(numNodes);
    auto getIndex = [cols](int r, int c) { return r * cols + c; };

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int current = getIndex(r, c);
            if (c + 1 < cols) {
                int right = getIndex(r, c + 1);
                graph[current].push_back({ right, 1.0, true });
                graph[right].push_back({ current, 1.0, true });
            }
            if (r + 1 < rows) {
                int down = getIndex(r + 1, c);
                graph[current].push_back({ down, 1.0, true });
                graph[down].push_back({ current, 1.0, true });
            }
        }
    }
    return graph;
}

vector<Intersection> generateGridPositions(int rows, int cols, int width, int height) {
    vector<Intersection> positions(rows * cols);
    float xSpacing = static_cast<float>(width) / (cols + 1);
    float ySpacing = static_cast<float>(height) / (rows + 1);

    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            positions[r * cols + c] = { (c + 1) * xSpacing, (r + 1) * ySpacing };
    return positions;
}

// draw Road
void drawThickLine(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, float thickness, sf::Color color) {
    sf::Vector2f direction = p2 - p1;
    float length = sqrt(direction.x * direction.x + direction.y * direction.y);
    float angle = atan2(direction.y, direction.x) * 180.0f / 3.14159265f;

    sf::RectangleShape road(sf::Vector2f(length, thickness));
    road.setPosition(p1);
    road.setRotation(angle);
    road.setFillColor(color);
    road.setOrigin(0, thickness / 2.0f);
    window.draw(road);
}

// distance from point p to segment ab
float distanceToLine(sf::Vector2f p, sf::Vector2f a, sf::Vector2f b) {
    float A = p.x - a.x;
    float B = p.y - a.y;
    float C = b.x - a.x;
    float D = b.y - a.y;

    float dot = A * C + B * D;
    float len_sq = C * C + D * D;
    float param = (len_sq != 0) ? (dot / len_sq) : -1;

    float xx, yy;
    if (param < 0) {
        xx = a.x; yy = a.y;
    }
    else if (param > 1) {
        xx = b.x; yy = b.y;
    }
    else {
        xx = a.x + param * C;
        yy = a.y + param * D;
    }

    float dx = p.x - xx;
    float dy = p.y - yy;
    return sqrt(dx * dx + dy * dy);
}

// remove a road (both directions)
void removeRoad(vector<vector<Road>>& graph, int i, int j) {
    for (auto& r : graph[i]) if (r.destination == j) r.active = false;
    for (auto& r : graph[j]) if (r.destination == i) r.active = false;
}

// remove one direction of the road, creating a one-way road
//to-do
void oneway(vector<vector<Road>>& graph, int i, int j) {};


// dijkstra algorithm to calculate the optimal path each car must take from their start node to their end node
// following the active roads only and updating in real time with the deletion of roads
vector<int> dijkstra(const vector<vector<Road>>& graph, int start, int goal) {
    int n = graph.size();
    vector<double> dist(n, numeric_limits<double>::infinity());
    vector<int> prev(n, -1);
    vector<bool> visited(n, false);
    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    dist[start] = 0;
    pq.push({ 0, start });

    while (!pq.empty()) {
        int u = pq.top().second; pq.pop();
        if (visited[u]) continue;
        visited[u] = true;
        if (u == goal) break;
        for (const auto& road : graph[u]) {
            if (!road.active) continue;
            int v = road.destination;
            double w = road.distance;
            if (dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                prev[v] = u;
                pq.push({ dist[v], v });
            }
        }
    }

    vector<int> path;
    if (prev[goal] == -1 && start != goal) {
        if (start == goal) { path.push_back(start); return path; }
    }
    int at = goal;
    while (at != -1) {
        path.push_back(at);
        at = prev[at];
    }
    reverse(path.begin(), path.end());
    if (path.size() == 1 && path[0] != start) path.clear();
    return path;
}

// cars
struct Car {
    int id;
    int startNode;
    int endNode;
    vector<int> path;      // sequence of nodes
    int pathIndex = 0;     // index of current node in path
    float progress = 0.f;  // 0..1 along current segment
    float speed = 120.f;   // pixels per second
    sf::Color color;
    sf::Vector2f position;
    bool onEdge = false;
    pair<int, int> reservedEdge = { -1,-1 }; // directed edge (a,b) or (-1,-1) if none

    int currentNode() const { return pathIndex; }
    int nextNode() const { return (pathIndex + 1 < (int)path.size()) ? path[pathIndex + 1] : -1; }
};

// for random
std::mt19937 rng((unsigned)std::random_device{}());
int randint(int a, int b) {
    std::uniform_int_distribution<int> d(a, b); return d(rng);
}
float randfloat(float a, float b) {
    std::uniform_real_distribution<float> d(a, b); return d(rng);
}

// integer key
inline long long edgeKey(int a, int b) {
    return ((long long)a << 32) | (unsigned int)b;
}

int main() {
    // configuration
    // to-do UI for configuration
    const int WIDTH = 1920, HEIGHT = 1080;
    const int ROWS = 7, COLS = 15;            // grid size (changeable) - will lag on giant grids - RECOMMENDED 7R,15C
    const int NUM_CARS = 150;                 // number of cars
    const float ROAD_THICKNESS = 25.0f;       // thickness of the roads
    const float CLICK_TOLERANCE = 10.0f;      //determines how close your click has to be to the target to perform the action (e.g. delete road)
    const float minSpacing = 30.f;           // pixels minimum following distance for cars

    // generate graph & positions
    auto graph = generateCityGrid(ROWS, COLS);
    int numNodes = ROWS * COLS;
    auto positions = generateGridPositions(ROWS, COLS, WIDTH, HEIGHT);

    sf::Clock timeClock;       // separate clock for simulation time
    float simTime = 6.f;       // simulated time in hours (24 hr format)
    // set starting hour (6AM default)
    const float SECONDS_PER_HOUR = 5.f; // seconds per hour, can be changed

    // load arial font
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        cerr << "Failed to load font\n";
    }

    sf::Text hudText;
    hudText.setFont(font);
    hudText.setCharacterSize(28);
    hudText.setFillColor(sf::Color::White);
    hudText.setPosition(20.f, 15.f);


    // precompute edge pixel lengths
    auto edgeLength = [&](int a, int b)->float {
        sf::Vector2f pa(positions[a].x, positions[a].y), pb(positions[b].x, positions[b].y);
        return sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
        };

    // window
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Urban Traffic v0.0.1");
    window.setFramerateLimit(60); //fps limit - 60 by default, works on other fps

    // visuals
    sf::CircleShape nodeShape(7); nodeShape.setOrigin(7, 7);
    sf::CircleShape carShape(6); carShape.setOrigin(6, 6);

    // directed edge occupancy: map from directed edge key (a->b) -> vector of (carId, progress)
    // progress is 0..1 along that directed segment
    unordered_map<long long, vector<pair<int, float>>> edgeOccupants;

    // initialize cars
    vector<Car> cars;
    cars.reserve(NUM_CARS);
    for (int i = 0; i < NUM_CARS; ++i) {
        int s = randint(0, numNodes - 1);
        int e = s;
        while (e == s) e = randint(0, numNodes - 1);
        Car car;
        car.id = i;
        car.startNode = s; //node at which the car spawns
        car.endNode = e; //destination node
        car.color = sf::Color((sf::Uint8)randint(50, 255), (sf::Uint8)randint(50, 255), (sf::Uint8)randint(50, 255));
        car.speed = randfloat(80.f, 150.f); // used for the variation of speeds
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


    //clock
    sf::Clock clock;
    int previewStart = -1, previewEnd = -1;
    vector<int> previewPath;

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();

            // left-click: toggle road (remove or re-add)
            // cars will ignore this until after theyve left the road
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y);
                for (int i = 0; i < numNodes; ++i) {
                    for (auto& r : graph[i]) {
                        int j = r.destination;
                        if (j > i) {
                            sf::Vector2f a(positions[i].x, positions[i].y), b(positions[j].x, positions[j].y);
                            if (distanceToLine(mouse, a, b) < CLICK_TOLERANCE) {
                                bool currentlyActive = r.active;

                                // toggle state
                                for (auto& rr : graph[i]) if (rr.destination == j) rr.active = !currentlyActive;
                                for (auto& rr : graph[j]) if (rr.destination == i) rr.active = !currentlyActive;

                                if (currentlyActive) {
                                    cout << "Deleted road: " << i << " <-> " << j << "\n";
                                    edgeOccupants.erase(edgeKey(i, j));
                                    edgeOccupants.erase(edgeKey(j, i));
                                }
                                else {
                                    cout << "Re-added road: " << i << " <-> " << j << "\n";
                                }
                            }
                        }
                    }
                }
            }


            // right-click: preview path - used for debugging, finds idel path using dijkstra
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right) {
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y);
                int nearest = -1; float minD = 30.f;
                for (int i = 0; i < numNodes; ++i) {
                    float dx = positions[i].x - mouse.x;
                    float dy = positions[i].y - mouse.y;
                    float d = sqrt(dx * dx + dy * dy);
                    if (d < minD) { minD = d; nearest = i; }
                }
                if (nearest != -1) {
                    if (previewStart == -1) previewStart = nearest;
                    else if (previewEnd == -1) {
                        previewEnd = nearest;
                        previewPath = dijkstra(graph, previewStart, previewEnd);
                        if (previewPath.empty()) cout << "No preview path available\n";
                        else cout << "Preview path computed\n";
                    }
                    else {
                        previewStart = nearest; previewEnd = -1; previewPath.clear();
                    }
                }
            }
        } // event loop

        // car update loop
        vector<int> order(cars.size());
        for (int i = 0; i < (int)cars.size(); ++i) order[i] = i;
        shuffle(order.begin(), order.end(), rng);

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
                    vector<int> newPath = dijkstra(graph, cur, car.endNode);
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
                float segLen = edgeLength(cur, nxt);

                bool canEnter = false;
                auto& vec = edgeOccupants[k];
                if (vec.empty()) {
                    canEnter = true;
                }
                else {
                    // find front-most car progress (max progress)
                    float frontProg = 0.f;
                    for (auto& p : vec) frontProg = max(frontProg, p.second);
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
                float segLen = sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
                if (segLen <= 0.001f) {
                    car.position = pb;
                    // remove occupant entry
                    long long k = edgeKey(a, b);
                    auto& vec = edgeOccupants[k];
                    vec.erase(remove_if(vec.begin(), vec.end(), [&](const pair<int, float>& p) { return p.first == car.id; }), vec.end());
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
                auto it = find_if(vec.begin(), vec.end(), [&](const pair<int, float>& p) { return p.first == car.id; });
                if (it == vec.end()) {
                    // should not happen; reinsert
                    vec.push_back({ car.id, car.progress });
                    it = find_if(vec.begin(), vec.end(), [&](const pair<int, float>& p) { return p.first == car.id; });
                }

                // find the smallest progress strictly greater than this car's progress (i.e., car ahead)
                float myProg = it->second;
                float nextAheadProg = 2.0f; // >1 means none
                for (auto& p : vec) {
                    if (p.first == car.id) continue;
                    if (p.second > myProg) nextAheadProg = min(nextAheadProg, p.second);
                }

                // compute allowed maximum progress considering spacing
                float allowedMaxProg = 1.0f;
                if (nextAheadProg <= 1.0f) {
                    float allowedDist = nextAheadProg * segLen - minSpacing;
                    allowedMaxProg = max(0.0f, allowedDist / segLen);
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
                    vec.erase(remove_if(vec.begin(), vec.end(), [&](const pair<int, float>& p) { return p.first == car.id; }), vec.end());
                    if (vec.empty()) edgeOccupants.erase(k);
                    car.onEdge = false;
                    car.reservedEdge = { -1,-1 };
                    car.pathIndex++;
                    car.progress = 0.f;
                }
                else {
                    // compute position along segment
                    sf::Vector2f dir = pb - pa;
                    float len = sqrt(dir.x * dir.x + dir.y * dir.y);
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
        } // end cars loop

        // rendering
        // time of day updates
        float realElapsed = timeClock.getElapsedTime().asSeconds();
        timeClock.restart();
        simTime += (realElapsed / SECONDS_PER_HOUR); // advance hours
        if (simTime >= 24.f) simTime -= 24.f;        // wrap around

        // compute hour & minute
        int hour = static_cast<int>(simTime);
        int minute = static_cast<int>((simTime - hour) * 60.f);

        // format time string
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d", hour, minute);
        hudText.setString(buf);

        window.clear(sf::Color(230, 230, 230));

        // draw roads
        for (int i = 0; i < numNodes; ++i) {
            for (auto& r : graph[i]) {
                int j = r.destination;
                if (j > i && r.active) {
                    drawThickLine(window, { positions[i].x, positions[i].y }, { positions[j].x, positions[j].y },
                        ROAD_THICKNESS, sf::Color(100, 100, 100));
                }
            }
        }

        // draw directed occupied edges with traffic-based color - shows which roads are the most stressed
        // needs expanding to roads longer than 1 node
        for (auto& entry : edgeOccupants) {
            long long key = entry.first;
            int a = (int)(key >> 32);
            int b = (int)(key & 0xFFFFFFFF);

            int carCount = entry.second.size();           // how many cars on this edge
            float t = std::min(1.f, carCount / 10.f);     // normalize traffic level (10 cars = full red) - currently not possible
            //sets road color
            sf::Color roadColor(
                (sf::Uint8)(100 + 155 * t),               
                (sf::Uint8)(100 * (1.f - t)),             
                (sf::Uint8)(100 * (1.f - t))              
            );

            drawThickLine(window,
                { positions[a].x, positions[a].y },
                { positions[b].x, positions[b].y },
                ROAD_THICKNESS + 2,
                roadColor
            );
        }


        // draw preview path (if chosen)
        if (!previewPath.empty()) {
            for (size_t i = 0; i + 1 < previewPath.size(); ++i) {
                drawThickLine(window, { positions[previewPath[i]].x, positions[previewPath[i]].y },
                    { positions[previewPath[i + 1]].x, positions[previewPath[i + 1]].y },
                    ROAD_THICKNESS / 1.5f, sf::Color::Red);
            }
        }

        // draw cars (above roads)
        for (auto& car : cars) {
            carShape.setPosition(car.position);
            carShape.setFillColor(car.color);
            carShape.setOutlineThickness(1.f);
            carShape.setOutlineColor(sf::Color::Black);
            window.draw(carShape);
        }

        // draw nodes - mostly debugging and not entirely necessary
        for (int i = 0; i < numNodes; ++i) {
            nodeShape.setPosition(positions[i].x, positions[i].y);
            nodeShape.setFillColor(sf::Color(0, 110, 255));
            window.draw(nodeShape);
        }
        // lighting
        float brightness = 1.0f;

        // determines background brightness using time of day to simulate day-night cycle (1 at noon 0.2 at midnight)
        brightness = 0.2f + 0.8f * max(0.f, cos((simTime - 12.f) / 24.f * 3.14159f * 2.f));

        sf::RectangleShape overlay(sf::Vector2f(WIDTH, HEIGHT));
        sf::Uint8 darkness = static_cast<sf::Uint8>((1.f - brightness) * 180.f); // max alpha = 180
        overlay.setFillColor(sf::Color(0, 0, 40, darkness));
        window.draw(overlay);

        // draw HUD text on top
        window.draw(hudText);


        window.display();
    } // main loop

    return 0;
}
