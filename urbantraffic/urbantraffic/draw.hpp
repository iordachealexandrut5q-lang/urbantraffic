#pragma once
#include <SFML/Graphics.hpp>

// draw Road
void drawThickLine(sf::RenderWindow& window, sf::Vector2f p1, sf::Vector2f p2, float thickness, sf::Color color);

// distance from point p to segment ab
float distanceToLine(sf::Vector2f p, sf::Vector2f a, sf::Vector2f b);
