#include <fstream>
#include <iomanip>
#include <vector>
#include <set>
#include <utility>
#include <iostream>

int main() {
    const int cols = 12;
    const int rows = 10;
    const int total = cols * rows;
    const double x0 = 100.0;
    const double y0 = 50.0;
    const double dx = 150.0;
    const double dy = 100.0;

    // build node positions (grid with slight offsets to make it "interesting")
    std::vector<std::pair<double,double>> pos;
    pos.reserve(total);
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double x = x0 + c * dx;
            double y = y0 + r * dy;
            // apply small perturbation to avoid a perfectly regular grid
            double perturbX = ((c * 7 + r * 13) % 17 - 8) * 2.3; // deterministic perturb
            double perturbY = ((c * 11 + r * 5) % 13 - 6) * 1.8;
            pos.emplace_back(x + perturbX, y + perturbY);
        }
    }

    // collect undirected edges (store as (min, max))
    std::set<std::pair<int,int>> edges;

    auto addEdge = [&](int a, int b) {
        if (a == b) return;
        if (a > b) std::swap(a,b);
        edges.insert({a,b});
    };

    // 1) Grid connections (horizontal + vertical)
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int i = r * cols + c;
            if (c + 1 < cols) addEdge(i, i + 1);       // right neighbor
            if (r + 1 < rows) addEdge(i, i + cols);    // down neighbor
        }
    }

    // 2) Diagonal "avenues" for visual variety
    for (int r = 0; r + 1 < rows; ++r) {
        for (int c = 0; c + 1 < cols; ++c) {
            int i = r * cols + c;
            // pattern-based diagonals
            if ((r + c) % 4 == 0) addEdge(i, i + cols + 1); // down-right
            if ((r + c) % 5 == 0 && c > 0) addEdge(i, i + cols - 1); // down-left
        }
    }

    // 3) A few long "highways" and radial links to/from center area
    int centerCol = cols / 2 - 1; // 5
    int centerRow = rows / 2;      // 5 -> choose one of central rows
    int center = centerRow * cols + centerCol; // central hub
    addEdge(center, 0);                 // top-left
    addEdge(center, cols - 1);          // top-right
    addEdge(center, total - cols);      // bottom-left (first col of last row)
    addEdge(center, total - 1);         // bottom-right
    // connect center to mid-edges
    addEdge(center, cols / 2);          // top middle
    addEdge(center, total - cols + cols / 2); // bottom middle
    addEdge(center, center - 2);
    addEdge(center, center + 3);

    // 4) Periodic cross-connections to create short circuits / small-world feel
    for (int r = 1; r + 1 < rows; r += 2) {
        for (int c = 0; c + 3 < cols; c += 3) {
            int i = r * cols + c;
            addEdge(i, i + 3);
            if (r + 2 < rows) addEdge(i, i + 2*cols + 1);
        }
    }

    // 5) A few plaza loops: small cycles inside blocks
    for (int r = 2; r + 2 < rows; r += 3) {
        for (int c = 2; c + 2 < cols; c += 4) {
            int a = r * cols + c;
            int b = a + 1;
            int d = a + cols;
            int e = d + 1;
            addEdge(a,b);
            addEdge(b,e);
            addEdge(e,d);
            addEdge(d,a);
        }
    }

    // write file
    const std::string filename = "urbantraffic\\saves\\citymap_120nodes.txt";
    std::ofstream out(filename);
    if (!out.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing\n";
        return 1;
    }

    out << std::fixed << std::setprecision(2);
    out << "=== CITY MAP EXPORT ===\n\n";

    for (size_t i = 0; i < pos.size(); ++i) {
        out << "NODE " << i << "\n";
        out << "X: " << pos[i].first << "   Y: " << pos[i].second << "\n\n";
    }

    out << "=== CONNECTIONS ===\n";
    for (const auto& e : edges) {
        out << "C: NODE " << e.first << " - NODE " << e.second << "\n";
    }
    out.close();

    std::cout << "Generated " << filename << " (" << pos.size() << " nodes, " << edges.size() << " edges)\n";
    return 0;
}