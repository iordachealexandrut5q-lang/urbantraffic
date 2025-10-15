// time.hpp
#pragma once
#include <SFML/System.hpp>
#include <string>

struct SimulationClock {
    sf::Clock clock;
    float simTime = 6.f; // start at 6am
    const float SECONDS_PER_HOUR = 5.f;
    void update();
    std::string getTimeString() const;
};
