#include "draw.hpp"
#include <cmath>

// draw Road
void drawThickLine(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, float thickness, sf::Color color) {
    sf::Vector2f direction = p2 - p1;
    float length = sqrt(direction.x * direction.x + direction.y * direction.y);
    float angle = atan2(direction.y, direction.x) * 180.0f / 3.14159265f;

    sf::RectangleShape road(sf::Vector2f(length, thickness));
    road.setPosition(p1);
    road.setRotation(angle);
    road.setFillColor(color);
    road.setOrigin(0, thickness / 2.0f);
    window.draw(road);
}

// distance from point p to segment ab
float distanceToLine(sf::Vector2f p, sf::Vector2f a, sf::Vector2f b) {
    float A = p.x - a.x;
    float B = p.y - a.y;
    float C = b.x - a.x;
    float D = b.y - a.y;

    float dot = A * C + B * D;
    float len_sq = C * C + D * D;
    float param = (len_sq != 0) ? (dot / len_sq) : -1;

    float xx, yy;
    if (param < 0) { xx = a.x; yy = a.y; }
    else if (param > 1) { xx = b.x; yy = b.y; }
    else { xx = a.x + param * C; yy = a.y + param * D; }

    float dx = p.x - xx;
    float dy = p.y - yy;
    return sqrt(dx * dx + dy * dy);
}
