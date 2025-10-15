#include "pathfinding.hpp"
#include <queue>
#include <limits>
#include <algorithm>

// dijkstra algorithm to calculate the optimal path each car must take from their start node to their end node
// following the active roads only and updating in real time with the deletion of roads
std::vector<int> dijkstra(const std::vector<std::vector<Road>>& graph, int start, int goal) {
    int n = graph.size();
    std::vector<double> dist(n, std::numeric_limits<double>::infinity());
    std::vector<int> prev(n, -1);
    std::vector<bool> visited(n, false);
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
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

    std::vector<int> path;
    if (prev[goal] == -1 && start != goal) {
        if (start == goal) { path.push_back(start); return path; }
    }
    int at = goal;
    while (at != -1) {
        path.push_back(at);
        at = prev[at];
    }
    std::reverse(path.begin(), path.end());
    if (path.size() == 1 && path[0] != start) path.clear();
    return path;
}
