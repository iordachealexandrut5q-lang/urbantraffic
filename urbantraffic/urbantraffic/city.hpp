#pragma once
#include <vector>
#include "road.hpp"

// generates a graph of nodes connected by roads in a rectangular grid
std::vector<std::vector<Road>> generateCityGrid(int rows, int cols);

std::vector<Intersection> generateGridPositions(int rows, int cols, int width, int height);
