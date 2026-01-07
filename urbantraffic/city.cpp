#include "city.hpp"
#include "utils.hpp"

// generate a grid-like city graph
std::vector<std::vector<Road>> City::generateCityGrid(int rows, int cols) {
	int numNodes = rows * cols; // total intersections
    std::vector<std::vector<Road>> graph(numNodes);
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

std::vector<Intersection> City::generateGridPositions(int rows, int cols, int width, int height) { 
    std::vector<Intersection> positions(rows * cols); 
    float xSpacing = static_cast<float>(width) / (cols + 1); 
    float ySpacing = static_cast<float>(height) / (rows + 1);

    for (int r = 0; r < rows; ++r) 
        for (int c = 0; c < cols; ++c)
            positions[r * cols + c] = { (c + 1) * xSpacing, (r + 1) * ySpacing };
    return positions;
}

// generate POIs based on fraction of total nodes
std::vector<int> City::generatePOIs(int rows, int cols, float fraction) {
    int numNodes = rows * cols;
    int target = std::max(1, (int)std::round(numNodes * fraction));
    target = std::max(1, target);

    std::vector<int> indices(numNodes);
    for (int i = 0; i < numNodes; ++i) indices[i] = i;
    std::shuffle(indices.begin(), indices.end(), Utils::rng);
    indices.resize(target);
    return indices;
}

City City::createGrid(int rows, int cols, int width, int height, float poiFraction) {
    City c;
    c.graph = generateCityGrid(rows, cols);
    c.positions = generateGridPositions(rows, cols, width, height);
    c.pois = generatePOIs(rows, cols, poiFraction);
    return c;
}
