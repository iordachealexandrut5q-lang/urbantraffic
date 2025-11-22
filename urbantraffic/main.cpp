//urban traffic simulator v 0.0.1
//read readme

#include <SFML/Graphics.hpp>
#include <iostream>
#include <unordered_map>
#include "road.hpp"
#include "city.hpp"
#include "draw.hpp"
#include "pathfinding.hpp"
#include "car.hpp"
#include "utils.hpp"
#include <fstream>
#include <iomanip>

using namespace std;

int main() {
    // configuration
    // to-do UI for configuration
    const int WIDTH = 1920, HEIGHT = 1080;
    const int ROWS = 7, COLS = 15;            // grid size (changeable) - will lag on giant grids - RECOMMENDED 7R,15C
    const int NUM_CARS = 150;                 // number of cars
    const float ROAD_THICKNESS = 25.0f;       // thickness of the roads
    const float CLICK_TOLERANCE = 10.0f;      // determines how close your click has to be to the target to perform the action (e.g. delete road)
    const float minSpacing = 30.f;            // pixels minimum following distance for cars
    const int minspeed = 80.f;                // car min speed (default 80)
    const int maxspeed = 150.f;               // car max speed (default 150)

    // generate graph & positions
    auto graph = generateCityGrid(ROWS, COLS);
    int numNodes = ROWS * COLS;
    auto positions = generateGridPositions(ROWS, COLS, WIDTH, HEIGHT);

    sf::Clock timeClock;       // separate clock for simulation time
    float simTime = 6.f;       // simulated time in hours (24 hr format)
    // set starting hour (6AM default)
    const float SECONDS_PER_HOUR = 5.f; // seconds per hour, can be changed

    // load arial font
    sf::Font font;
    if (!font.loadFromFile("arial.ttf")) {
        cerr << "Failed to load font\n";
    }

    sf::Text hudText;
    hudText.setFont(font);
    hudText.setCharacterSize(28);
    hudText.setFillColor(sf::Color::White);
    hudText.setPosition(20.f, 15.f);

    bool draggingNode = false;
    int draggedNodeIndex = -1;
    sf::Vector2f dragOffset;

    // precompute edge pixel lengths
    auto edgeLength = [&](int a, int b)->float {
        sf::Vector2f pa(positions[a].x, positions[a].y), pb(positions[b].x, positions[b].y);
        return sqrt((pa.x - pb.x) * (pa.x - pb.x) + (pa.y - pb.y) * (pa.y - pb.y));
        };

    // window
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Urban Traffic v0.0.1");
    window.setFramerateLimit(60); //fps limit - 60 by default, works on other fps

    // visuals
    sf::CircleShape nodeShape(7); nodeShape.setOrigin(7, 7);
    sf::CircleShape carShape(6); carShape.setOrigin(6, 6);

    // directed edge occupancy: map from directed edge key (a->b) -> vector of (carId, progress)
    // progress is 0..1 along that directed segment
    unordered_map<long long, vector<pair<int, float>>> edgeOccupants;

    // initialize cars using helper
    vector<Car> cars = initCars(NUM_CARS, graph, positions, minspeed, maxspeed);

    //clock
    sf::Clock clock;
    int previewStart = -1, previewEnd = -1;
    vector<int> previewPath;

    while (window.isOpen()) {

        float dt = clock.restart().asSeconds();
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();

            // left-click: toggle road (remove or re-add)
            // cars will ignore this until after theyve left the road
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y);
                for (int i = 0; i < numNodes; ++i) {
                    for (auto& r : graph[i]) {
                        int j = r.destination;
                        if (j > i) {
                            sf::Vector2f a(positions[i].x, positions[i].y), b(positions[j].x, positions[j].y);
                            if (distanceToLine(mouse, a, b) < CLICK_TOLERANCE) {
                                bool currentlyActive = r.active;

                                // toggle state
                                for (auto& rr : graph[i]) if (rr.destination == j) rr.active = !currentlyActive;
                                for (auto& rr : graph[j]) if (rr.destination == i) rr.active = !currentlyActive;

                                if (currentlyActive) {
                                    cout << "Deleted road: " << i << " <-> " << j << "\n";
                                    edgeOccupants.erase(edgeKey(i, j));
                                    edgeOccupants.erase(edgeKey(j, i));
                                }
                                else {
                                    cout << "Re-added road: " << i << " <-> " << j << "\n";
                                }
                            }
                        }
                    }
                }
            }

            // ress 'S' to save map to file
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::S) {
                saveCityToFile(graph, positions, "saves/citymap.txt");

            }

            // Press 'L' to load map from file
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::L) {
                graph = loadCityFromFile("saves/citymap.txt", positions);
            }

            // Middle-click or Shift+Right-click to drag nodes
            if (event.type == sf::Event::MouseButtonPressed &&
                (event.mouseButton.button == sf::Mouse::Middle ||
                    (event.mouseButton.button == sf::Mouse::Right && sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))))
            {
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y);
                for (int i = 0; i < numNodes; ++i) {
                    float dx = positions[i].x - mouse.x;
                    float dy = positions[i].y - mouse.y;
                    float dist = sqrt(dx * dx + dy * dy);
                    if (dist < 12.f) { // within clickable radius
                        draggingNode = true;
                        draggedNodeIndex = i;
                        dragOffset = { positions[i].x - mouse.x, positions[i].y - mouse.y };
                        break;
                    }
                }
            }

            if (event.type == sf::Event::MouseMoved && draggingNode && draggedNodeIndex != -1) {
                sf::Vector2f mouse(event.mouseMove.x, event.mouseMove.y);
                positions[draggedNodeIndex].x = mouse.x + dragOffset.x;
                positions[draggedNodeIndex].y = mouse.y + dragOffset.y;
            }

            if (event.type == sf::Event::MouseButtonReleased) {
                draggingNode = false;
                draggedNodeIndex = -1;
            }




            // right-click: preview path - used for debugging, finds idel path using dijkstra
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right) {
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y);
                int nearest = -1; float minD = 30.f;
                for (int i = 0; i < numNodes; ++i) {
                    float dx = positions[i].x - mouse.x;
                    float dy = positions[i].y - mouse.y;
                    float d = sqrt(dx * dx + dy * dy);
                    if (d < minD) { minD = d; nearest = i; }
                }
                if (nearest != -1) {
                    if (previewStart == -1) previewStart = nearest;
                    else if (previewEnd == -1) {
                        previewEnd = nearest;
                        previewPath = dijkstra(graph, previewStart, previewEnd);
                        if (previewPath.empty()) cout << "No preview path available\n";
                        else cout << "Preview path computed\n";
                    }
                    else {
                        previewStart = nearest; previewEnd = -1; previewPath.clear();
                    }
                }
            }
        } // event loop

        // update cars
        updateCars(cars, graph, positions, edgeOccupants, dt, minSpacing, ROAD_THICKNESS);

        // rendering
        // time of day updates
        float realElapsed = timeClock.getElapsedTime().asSeconds();
        timeClock.restart();
        simTime += (realElapsed / SECONDS_PER_HOUR); // advance hours
        if (simTime >= 24.f) simTime -= 24.f;        // wrap around

        // compute hour & minute
        int hour = static_cast<int>(simTime);
        int minute = static_cast<int>((simTime - hour) * 60.f);

        // format time string
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d", hour, minute);
        hudText.setString(buf);

        window.clear(sf::Color(230, 230, 230));

        // draw roads
        for (int i = 0; i < numNodes; ++i) {
            for (auto& r : graph[i]) {
                int j = r.destination;
                if (j > i && r.active) {
                    drawThickLine(window, { positions[i].x, positions[i].y }, { positions[j].x, positions[j].y },
                        ROAD_THICKNESS, sf::Color(100, 100, 100));
                }
            }
        }

        // draw directed occupied edges with traffic-based color - shows which roads are the most stressed
        // needs expanding to roads longer than 1 node
        for (auto& entry : edgeOccupants) {
            long long key = entry.first;
            int a = (int)(key >> 32);
            int b = (int)(key & 0xFFFFFFFF);

            int carCount = entry.second.size();           // how many cars on this edge
            float t = std::min(1.f, carCount / 10.f);     // normalize traffic level (10 cars = full red) - currently not possible
            //sets road color
            sf::Color roadColor(
                (sf::Uint8)(100 + 155 * t),               
                (sf::Uint8)(100 * (1.f - t)),             
                (sf::Uint8)(100 * (1.f - t))              
            );

            drawThickLine(window,
                { positions[a].x, positions[a].y },
                { positions[b].x, positions[b].y },
                ROAD_THICKNESS + 2,
                roadColor
            );
        }


        // draw preview path (if chosen)
        if (!previewPath.empty()) {
            for (size_t i = 0; i + 1 < previewPath.size(); ++i) {
                drawThickLine(window, { positions[previewPath[i]].x, positions[previewPath[i]].y },
                    { positions[previewPath[i + 1]].x, positions[previewPath[i + 1]].y },
                    ROAD_THICKNESS / 1.5f, sf::Color::Red);
            }
        }

        // draw cars (above roads)
        for (auto& car : cars) {
            carShape.setPosition(car.position);
            carShape.setFillColor(car.color);
            carShape.setOutlineThickness(1.f);
            carShape.setOutlineColor(sf::Color::Black);
            window.draw(carShape);
        }

        // draw nodes - mostly debugging and not entirely necessary
        for (int i = 0; i < numNodes; ++i) {
            nodeShape.setPosition(positions[i].x, positions[i].y);
            nodeShape.setFillColor(sf::Color(0, 110, 255));
            window.draw(nodeShape);
        }
        // lighting
        float brightness = 1.0f;

        // determines background brightness using time of day to simulate day-night cycle (1 at noon 0.2 at midnight)
        brightness = 0.2f + 0.8f * max(0.f, cos((simTime - 12.f) / 24.f * 3.14159f * 2.f));

        sf::RectangleShape overlay(sf::Vector2f(WIDTH, HEIGHT));
        sf::Uint8 darkness = static_cast<sf::Uint8>((1.f - brightness) * 180.f); // max alpha = 180
        overlay.setFillColor(sf::Color(0, 0, 40, darkness));
        window.draw(overlay);

        // draw HUD text on top
        window.draw(hudText);

        window.display();
    } // main loop

    return 0;
}
