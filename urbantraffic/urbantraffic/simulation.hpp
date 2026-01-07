#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <unordered_map>

#include "road.hpp"
#include "car.hpp"
#include "city.hpp"

class Simulation {
public:
    Simulation(int width, int height,
        int rows, int cols,
        int numCars,
        float roadThickness,
        float clickTolerance,
        float minSpacing,
        int minSpeed,
        int maxSpeed,
        float secondsPerHour);

    int run();

private:
    // configuration
    int WIDTH, HEIGHT;
    int ROWS, COLS;
    int NUM_CARS;
    float ROAD_THICKNESS;
    float CLICK_TOLERANCE;
    float MIN_SPACING;
    int MIN_SPEED, MAX_SPEED;
    float SECONDS_PER_HOUR;

    // simulation state
    std::vector<std::vector<Road>> graph;
    std::vector<Intersection> positions;
    std::unordered_map<long long, std::vector<std::pair<int, float>>> edgeOccupants;
    std::vector<Car> cars;

    // rendering / UI
    sf::Font font;
    sf::Text hudText;

    // helpers
    void handleEvents(sf::RenderWindow& window, bool& running, bool& draggingNode, int& draggedNodeIndex, sf::Vector2f& dragOffset,
        int& previewStart, int& previewEnd, std::vector<int>& previewPath, sf::Clock& timeClock, float& simTime, sf::Clock& clock);
    void update(float dt);
    void render(sf::RenderWindow& window, sf::Clock& timeClock, float& simTime, int previewStart, int previewEnd, const std::vector<int>& previewPath);
};
