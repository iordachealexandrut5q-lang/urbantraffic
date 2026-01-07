#include "simulation.hpp"
#include "utils.hpp"
#include "car.hpp"
#include "city.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <sstream>

Simulation::Simulation(int width, int height,
    int rows, int cols,
    int numCars,
    float roadThickness,
    float clickTolerance,
    float minSpacing,
    int minSpeed,
    int maxSpeed,
    float secondsPerHour,
    float poichance,
    int busMinSpeed,
    int busMaxSpeed,
    bool commuteEnabled)
    : WIDTH(width), HEIGHT(height), ROWS(rows), COLS(cols), NUM_CARS(numCars),
    ROAD_THICKNESS(roadThickness), CLICK_TOLERANCE(clickTolerance), MIN_SPACING(minSpacing),
    MIN_SPEED(minSpeed), MAX_SPEED(maxSpeed), SECONDS_PER_HOUR(secondsPerHour),
    poichance(poichance), COMMUTE_ENABLED(commuteEnabled) {

    // generate graph & positions
    City c = City::createGrid(ROWS, COLS, WIDTH, HEIGHT, 0.05f);
    graph = std::move(c.graph);
    positions = std::move(c.positions);
    pois = std::move(c.pois);

    // load font
    if (!font.loadFromFile("arial.ttf")) {
        std::cerr << "Failed to load font\n";
    }
    hudText.setFont(font);
    hudText.setCharacterSize(28);
    hudText.setFillColor(sf::Color::White);
    hudText.setPosition(20.f, 15.f);

    // set initial speed limit to MAX_SPEED
    speedLimit = MAX_SPEED;

    // initialize cars
    cars = Car::initCars(NUM_CARS, graph, positions, MIN_SPEED, MAX_SPEED, COMMUTE_ENABLED);
    // initialize a few buses using provided ranges
    buses = Bus::initBuses(std::max(1, NUM_CARS / 10), graph, positions, busMinSpeed, busMaxSpeed, NUM_CARS, 3);
}

int Simulation::run() {
    sf::Clock timeClock;
    float simTime = 6.f; // start at 6:00 AM

    // window
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Urban Traffic v0.0.2");
    window.setFramerateLimit(60);

    // visuals
    sf::CircleShape nodeShape(7); nodeShape.setOrigin(7, 7); // intersections
    sf::CircleShape carShape(6); carShape.setOrigin(6, 6); // cars
    sf::RectangleShape busShape(sf::Vector2f(12, 12)); busShape.setOrigin(6, 6); // buses as squares

    // clock
    sf::Clock clock;

    bool draggingNode = false; // node dragging state
    int draggedNodeIndex = -1; // index of dragged node
    sf::Vector2f dragOffset; // offset from mouse to node center

    // input mode for setting speed limit
    bool enteringSpeed = false;
    std::string speedInput;

    while (window.isOpen()) {
        float dt = clock.restart().asSeconds();

        // event handling
        sf::Event event;
        while (window.pollEvent(event)) {
            // event loop
            if (event.type == sf::Event::Closed) window.close();

            // text entry when setting speed
            if (enteringSpeed) {
                if (event.type == sf::Event::TextEntered) {
                    if (event.text.unicode >= '0' && event.text.unicode <= '9') {
                        speedInput.push_back(static_cast<char>(event.text.unicode));
                    }
                    else if (event.text.unicode == '\b' && !speedInput.empty()) { // backspace
                        speedInput.pop_back();
                    }
                    else if (event.text.unicode == '\r' || event.text.unicode == '\n') { // enter commit
                        if (!speedInput.empty()) {
                            try {
                                int newLimit = std::stoi(speedInput);
                                if (newLimit > 0) {
                                    speedLimit = newLimit;
                                    // clamp car desired speeds
                                    for (auto& car : cars) {
                                        if (car.speed > (float)speedLimit) car.speed = (float)speedLimit;
                                        if (car.velocity > (float)speedLimit) car.velocity = (float)speedLimit;
                                    }
                                    // clamp bus desired speeds
                                    for (auto& bus : buses) {
                                        if (bus.speed > (float)speedLimit) bus.speed = (float)speedLimit;
                                        if (bus.velocity > (float)speedLimit) bus.velocity = (float)speedLimit;
                                    }
                                    std::cout << "Speed limit set to " << speedLimit << "\n";
                                }
                            } catch (...) { }
                        }
                        enteringSpeed = false;
                        speedInput.clear();
                    }
                    else if (event.text.unicode == 27) { // ESC cancel
                        enteringSpeed = false;
                        speedInput.clear();
                    }
                }
                // while entering, skip other event handling
                continue;
            }

            // left-click: toggle road
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y);
                int numNodes = ROWS * COLS;
                for (int i = 0; i < numNodes; ++i) {
                    for (auto& r : graph[i]) {
                        int j = r.destination; 
                        if (j > i) {
                            sf::Vector2f a(positions[i].x, positions[i].y), b(positions[j].x, positions[j].y);
                            if (Utils::distanceToLine(mouse, a, b) < CLICK_TOLERANCE) {
                                bool currentlyActive = r.active;
                                for (auto& rr : graph[i]) if (rr.destination == j) rr.active = !currentlyActive;
                                for (auto& rr : graph[j]) if (rr.destination == i) rr.active = !currentlyActive;
                                if (currentlyActive) {
                                    std::cout << "Deleted road: " << i << " <-> " << j << "\n";
                                    edgeOccupants.erase(Utils::edgeKey(i, j));
                                    edgeOccupants.erase(Utils::edgeKey(j, i));
                                }
                                else {
                                    std::cout << "Re-added road: " << i << " <-> " << j << "\n";
                                }
                            }
                        }
                    }
                }
            }

			// save press S
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::S) {
                Utils::saveCityToFile(graph, positions, pois, "saves/citymap.txt");
            }

			// load press L
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::L) {
                graph = Utils::loadCityFromFile("saves/citymap.txt", positions, pois);
            }

            // press W to enter speed limit
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::W) {
                enteringSpeed = true;
                speedInput.clear();
                std::cout << "Enter new speed limit: " << std::flush;
            }

            // drag nodes
            if (event.type == sf::Event::MouseButtonPressed &&
                (event.mouseButton.button == sf::Mouse::Middle ||
                    (event.mouseButton.button == sf::Mouse::Right && sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)))) { 
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y); 
                int numNodes = ROWS * COLS; 
                for (int i = 0; i < numNodes; ++i) { // check if mouse is near node
                    float dx = positions[i].x - mouse.x;
                    float dy = positions[i].y - mouse.y;
					float dist = std::sqrt(dx * dx + dy * dy); // distance to node
                    if (dist < 12.f) {
                        draggingNode = true;
                        draggedNodeIndex = i;
                        dragOffset = { positions[i].x - mouse.x, positions[i].y - mouse.y };
                        break;
                    }
                }
            }

            if (event.type == sf::Event::MouseMoved && draggingNode && draggedNodeIndex != -1) { // update dragged node position
                sf::Vector2f mouse(event.mouseMove.x, event.mouseMove.y);
                positions[draggedNodeIndex].x = mouse.x + dragOffset.x;
                positions[draggedNodeIndex].y = mouse.y + dragOffset.y;
            }

            if (event.type == sf::Event::MouseButtonReleased) { // stop dragging
                draggingNode = false;
                draggedNodeIndex = -1;
            }

            // right-click: toggle POI for nearest node
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Right) {
                sf::Vector2f mouse(event.mouseButton.x, event.mouseButton.y);
                int numNodes = ROWS * COLS;
                int nearest = -1; float minD = 30.f;
                for (int i = 0; i < numNodes; ++i) {
                    float dx = positions[i].x - mouse.x;
                    float dy = positions[i].y - mouse.y;
                    float d = std::sqrt(dx * dx + dy * dy);
                    if (d < minD) { minD = d; nearest = i; }
                }
                if (nearest != -1) {
                    auto it = std::find(pois.begin(), pois.end(), nearest);
                    if (it == pois.end()) {
                        // add POI
                        pois.push_back(nearest);
                        std::cout << "Added POI: " << nearest << "\n";
                    } else {
                        // remove POI
                        pois.erase(it);
                        std::cout << "Removed POI: " << nearest << "\n";
                    }
                }
            }
        }

        // update cars (pass POIs and chance)
        Car::update(cars, graph, positions, edgeOccupants, dt, MIN_SPACING, ROAD_THICKNESS, pois, poichance, COMMUTE_ENABLED, simTime);
        // update buses
        Bus::update(buses, graph, positions, edgeOccupants, dt, MIN_SPACING, ROAD_THICKNESS);

        // rendering/time updates
        float realElapsed = timeClock.getElapsedTime().asSeconds(); // real time elapsed
        timeClock.restart();
        simTime += (realElapsed / SECONDS_PER_HOUR); // advance simulation time
        if (simTime >= 24.f) simTime -= 24.f; // wrap around after 24 hours

        // compute hour & minute
        int hour = static_cast<int>(simTime);
        int minute = static_cast<int>((simTime - hour) * 60.f); // convert fractional hour to minutes
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d", hour, minute); // format time as HH:MM
        // show speed limit in HUD
        std::ostringstream oss;
        oss << buf << "  |  SpeedLimit: " << speedLimit;
        hudText.setString(oss.str()); // update HUD text

        window.clear(sf::Color(230, 230, 230)); // light gray background

		int numNodes = ROWS * COLS; // total nodes
        // draw roads
        for (int i = 0; i < numNodes; ++i) {
            for (auto& r : graph[i]) {
                int j = r.destination;
                if (j > i && r.active) {
                    Utils::drawRoad(window, { positions[i].x, positions[i].y }, { positions[j].x, positions[j].y },
                        ROAD_THICKNESS, sf::Color(100, 100, 100));
                }
            }
        }

        // draw occupied edges colored by traffic
        for (auto& entry : edgeOccupants) {
            long long key = entry.first; // edge key
            int a = (int)(key >> 32); // source node
            int b = (int)(key & 0xFFFFFFFF); // destination node
            int carCount = entry.second.size(); // number of cars on this edge
			float t = std::min(1.f, carCount / 10.f); // traffic intensity out of 10 cars
            sf::Color roadColor(
                (sf::Uint8)(100 + 155 * t),
                (sf::Uint8)(100 * (1.f - t)),
                (sf::Uint8)(100 * (1.f - t))
            );
            Utils::drawRoad(window, { positions[a].x, positions[a].y }, { positions[b].x, positions[b].y }, ROAD_THICKNESS + 2, roadColor);
        }

        // draw nodes
        for (int i = 0; i < numNodes; ++i) {
            nodeShape.setPosition(positions[i].x, positions[i].y);
            // if node is a POI, draw red
            bool isPoi = std::find(pois.begin(), pois.end(), i) != pois.end();
            if (isPoi) nodeShape.setFillColor(sf::Color::Red);
            else nodeShape.setFillColor(sf::Color(100, 100, 255));
            window.draw(nodeShape);
        }

        // draw cars
        for (auto& car : cars) {
            carShape.setPosition(car.position);
            carShape.setFillColor(car.color);
            carShape.setOutlineThickness(1.f);
            carShape.setOutlineColor(sf::Color::Black);
            window.draw(carShape);
        }

        // draw buses as squares
        for (auto& bus : buses) {
            busShape.setPosition(bus.position);
            busShape.setFillColor(bus.color);
            busShape.setOutlineThickness(1.f);
            busShape.setOutlineColor(sf::Color::Black);
            window.draw(busShape);
        }

        // lighting overlay
        float brightness = 0.2f + 0.8f * std::max(0.f, std::cos((simTime - 12.f) / 24.f * 3.14159f * 2.f));
        sf::RectangleShape overlay(sf::Vector2f(WIDTH, HEIGHT));
        sf::Uint8 darkness = static_cast<sf::Uint8>((1.f - brightness) * 180.f);
        overlay.setFillColor(sf::Color(0, 0, 40, darkness));
        window.draw(overlay);

        // hud
        window.draw(hudText);
        window.display();
    }

    return 0;
}
