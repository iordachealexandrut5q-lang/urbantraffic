// time.cpp
#include "time.hpp"
#include <cstdio>

void SimulationClock::update() {
    float realElapsed = clock.restart().asSeconds();
    simTime += (realElapsed / SECONDS_PER_HOUR);
    if (simTime >= 24.f) simTime -= 24.f;
}

std::string SimulationClock::getTimeString() const {
    int hour = static_cast<int>(simTime);
    int minute = static_cast<int>((simTime - hour) * 60.f);
    char buf[16];
    snprintf(buf, sizeof(buf), "%02d:%02d", hour, minute);
    return buf;
}
