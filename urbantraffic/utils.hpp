#pragma once

#include <vector>
#include <string>
#include "road.hpp"
#include <SFML/Graphics.hpp>
#include <random>

class Utils {
public:
    static int randint(int a, int b);
    static float randfloat(float a, float b);

    static std::vector<std::vector<Road>> loadCityFromFile(const std::string& filename,
        std::vector<Intersection>& positions,
        std::vector<int>& pois);

    static void saveCityToFile(const std::vector<std::vector<Road>>& graph,
        const std::vector<Intersection>& positions,
        const std::vector<int>& pois,
        const std::string& filename);

    static std::vector<int> dijkstra(const std::vector<std::vector<Road>>& graph, int start, int goal);

    static void drawRoad(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, float thickness, sf::Color color);

    static float distanceToLine(sf::Vector2f p, sf::Vector2f a, sf::Vector2f b);

    static long long edgeKey(int a, int b);

	// random number generator
    static std::mt19937 rng;
};

// inline implementations
inline int randint(int a, int b) { return Utils::randint(a, b); }
inline float randfloat(float a, float b) { return Utils::randfloat(a, b); }

inline std::vector<std::vector<Road>> loadCityFromFile(const std::string& filename, std::vector<Intersection>& positions, std::vector<int>& pois) {
    return Utils::loadCityFromFile(filename, positions, pois);
}

inline void saveCityToFile(const std::vector<std::vector<Road>>& graph, const std::vector<Intersection>& positions, const std::vector<int>& pois, const std::string& filename) {
    Utils::saveCityToFile(graph, positions, pois, filename);
}

inline long long edgeKey(int a, int b) { return Utils::edgeKey(a, b); }