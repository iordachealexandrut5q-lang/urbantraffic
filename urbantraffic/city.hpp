#pragma once
#include <vector>
#include "road.hpp"

class City {
  public:
    std::vector<std::vector<Road>> graph;
    std::vector<Intersection> positions;
    std::vector<int> pois;

    City() = default;

    // create a city grid with positions and POIs
    static City createGrid(int rows, int cols, int width, int height,
                           float poiFraction = 0.05f);

    // legacy helpers (now as static members)
    static std::vector<std::vector<Road>> generateCityGrid(int rows, int cols);
    static std::vector<Intersection> generateGridPositions(int rows, int cols,
                                                           int width, int height);
    static std::vector<int> generatePOIs(int rows, int cols, float fraction = 0.05f);
};
