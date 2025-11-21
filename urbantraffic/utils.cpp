#include "utils.hpp"
#include <random>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>

std::mt19937 rng((unsigned)std::random_device{}());

int randint(int a, int b) {
    std::uniform_int_distribution<int> d(a, b); return d(rng);
}

float randfloat(float a, float b) {
    std::uniform_real_distribution<float> d(a, b); return d(rng);
}

std::vector<std::vector<Road>> loadCityFromFile(const std::string& filename,
    std::vector<Intersection>& positions) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        std::cerr << "Failed to open " << filename << "\n";
        return {};
    }

    std::vector<std::vector<Road>> graph;
    positions.clear();

    std::string token;
    while (in >> token) {
        if (token == "NODE") {
            int id;
            in >> id;
            std::string tmp;
            float x, y;
            in >> tmp >> x >> tmp >> y;
            if (id >= (int)positions.size()) positions.resize(id + 1);
            positions[id] = { x, y };
        }
        else if (token == "C:") {
            std::string n1, n2, dash;
            int a, b;
            in >> n1 >> a >> dash >> n2 >> b;
            if (graph.size() <= std::max(a, b)) graph.resize(std::max(a, b) + 1);
            graph[a].push_back({ b, 1.0, true });
            graph[b].push_back({ a, 1.0, true });
        }
    }

    return graph;
}


// save the current city graph and positions to a text file
void saveCityToFile(const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
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