#include "utils.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>

std::mt19937 Utils::rng((unsigned)std::random_device{}());


int Utils::randint(int a, int b) {
    std::uniform_int_distribution<int> d(a, b);
    return d(rng);
}

float Utils::randfloat(float a, float b) {
    std::uniform_real_distribution<float> d(a, b);
    return d(rng);
}
// generate a unique key for an edge (a,b)
long long Utils::edgeKey(int a, int b) {
    return (static_cast<long long>(a) << 32) | static_cast<unsigned int>(b);
}

// loads a city map from a file
std::vector<std::vector<Road>> Utils::loadCityFromFile(const std::string& filename,
    std::vector<Intersection>& positions,
    std::vector<int>& pois) {
	std::ifstream in(filename); // open file for reading - will be citymap.txt
    if (!in.is_open()) {
        std::cerr << "Failed to open " << filename << "\n"; 
        return {};
    }

    std::vector<std::vector<Road>> graph; 
    positions.clear(); 
    pois.clear(); 

    std::string token; 
    while (in >> token) {
        if (token == "NODE") { 
            int id;
			in >> id; // read node id
            std::string tmp; 
			float x, y; // read coordinates
            in >> tmp >> x >> tmp >> y; 
            if (id >= (int)positions.size()) positions.resize(id + 1); 
			positions[id] = { x, y }; // store intersection
        }
        else if (token == "C:") {
            std::string n1, n2, dash;
            int a, b;
            in >> n1 >> a >> dash >> n2 >> b;
            if (graph.size() <= std::max(a, b)) graph.resize(std::max(a, b) + 1);
            graph[a].push_back({ b, 1.0, true });
            graph[b].push_back({ a, 1.0, true });
        }
        else if (token == "POI") { 
			int id; in >> id; pois.push_back(id); // read POI id
        }
    }

    return graph;
}

void Utils::saveCityToFile(const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    const std::vector<int>& pois,
    const std::string& filename) {
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return;
    }

    out << std::fixed << std::setprecision(2);
    out << "=== CITY MAP EXPORT ===\n";

    // Write all nodes and their coordinates
    for (size_t i = 0; i < positions.size(); ++i) {
        out << "NODE " << i << "\n";
        out << "X: " << positions[i].x << "   Y: " << positions[i].y << "\n\n";
    }

    // Write POIs
    if (!pois.empty()) {
        out << "=== POIS ===\n";
        for (int p : pois) out << "POI " << p << "\n";
        out << "\n";
    }

    out << "=== CONNECTIONS ===\n";

    // Write all active connections (only once per undirected edge)
    for (size_t i = 0; i < graph.size(); ++i) {
        for (const auto& r : graph[i]) {
            if (r.active && r.destination > static_cast<int>(i)) {
                out << "C: NODE " << i << " - NODE " << r.destination << "\n";
            }
        }
    }

    out.close();
    std::cout << "Map successfully saved to " << filename << "\n";
}
// Dijkstra's algorithm to find shortest path from start to goal
// modified for the purpose of urban traffic simulation
std::vector<int> Utils::dijkstra(const std::vector<std::vector<Road>>& graph, int start, int goal) {
    int n = (int)graph.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);
    std::vector<bool> visited(n, false);
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    if (start < 0 || start >= n || goal < 0 || goal >= n) return {};

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
	// reconstruct path
    std::vector<int> path;
    if (goal < 0 || goal >= n) return path;
    if (prev[goal] == -1 && start != goal) {
        if (start == goal) { path.push_back(start); return path; }
    }
	int at = goal; // start from goal
	while (at != -1) { // backtrack to start
        path.push_back(at);
        at = prev[at];
    }
	std::reverse(path.begin(), path.end()); // reverse to get correct order
    if (path.size() == 1 && path[0] != start) path.clear(); 
    return path; 
}

// draws a road as a rectangle between two points
void Utils::drawRoad(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, float thickness, sf::Color color) { 
    sf::Vector2f direction = p2 - p1;
    float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    float angle = std::atan2(direction.y, direction.x) * 180.0f / 3.14159265f;

    sf::RectangleShape road(sf::Vector2f(length, thickness));
    road.setPosition(p1);
    road.setRotation(angle);
    road.setFillColor(color);
    road.setOrigin(0, thickness / 2.0f);
    window.draw(road);
}

float Utils::distanceToLine(sf::Vector2f p, sf::Vector2f a, sf::Vector2f b) {
	float A = p.x - a.x; 
    float B = p.y - a.y; 
    float C = b.x - a.x;
    float D = b.y - a.y;
	// dot product of AP and AB

	float dot = A * C + B * D; // length squared of AB
	float len_sq = C * C + D * D; // parameter along line
    float param = (len_sq != 0) ? (dot / len_sq) : -1; 

    float xx, yy; 
    if (param < 0) { xx = a.x; yy = a.y; } 
    else if (param > 1) { xx = b.x; yy = b.y; }
    else { xx = a.x + param * C; yy = a.y + param * D; }

    float dx = p.x - xx;
    float dy = p.y - yy;
	return std::sqrt(dx * dx + dy * dy); // return distance
}