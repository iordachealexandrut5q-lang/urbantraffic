#include "city.hpp"

// generates a graph of nodes connected by roads in a rectangular grid
std::vector<std::vector<Road>> generateCityGrid(int rows, int cols) {
    int numNodes = rows * cols;
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

std::vector<Intersection> generateGridPositions(int rows, int cols, int width, int height) {
    std::vector<Intersection> positions(rows * cols);
    float xSpacing = static_cast<float>(width) / (cols + 1);
    float ySpacing = static_cast<float>(height) / (rows + 1);

    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            positions[r * cols + c] = { (c + 1) * xSpacing, (r + 1) * ySpacing };
    return positions;
}
