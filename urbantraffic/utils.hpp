#pragma once
#include <random>
#include <vector>
#include <string>
#include "road.hpp"
#include "city.hpp"

// for random
extern std::mt19937 rng;
int randint(int a, int b);
float randfloat(float a, float b);

// save/load city
std::vector<std::vector<Road>> loadCityFromFile(const std::string& filename,
    std::vector<Intersection>& positions);

void saveCityToFile(const std::vector<std::vector<Road>>& graph,
    const std::vector<Intersection>& positions,
    const std::string& filename);

// integer key
inline long long edgeKey(int a, int b) {
    return ((long long)a << 32) | (unsigned int)b;
}