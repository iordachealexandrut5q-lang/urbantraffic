#pragma once

#include "utils.hpp"

inline void drawRoad(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, float thickness, sf::Color color) {
    Utils::drawRoad(window, p1, p2, thickness, color);
}

inline float distanceToLine(sf::Vector2f p, sf::Vector2f a, sf::Vector2f b) {
    return Utils::distanceToLine(p, a, b);
}
