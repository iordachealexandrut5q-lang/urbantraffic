//urban traffic simulator v 0.0.2

#include <SFML/Graphics.hpp>
#include <iostream>
#include <unordered_map>
#include "road.hpp"
#include "city.hpp"
#include "draw.hpp"
#include "car.hpp"
#include "utils.hpp"
#include <fstream>
#include <iomanip>
#include "simulation.hpp"

using namespace std;

int main() {
    const int WIDTH = 1920, HEIGHT = 1080;
    const int ROWS = 7, COLS = 15;            // grid size (changeable) - will lag on giant grids - RECOMMENDED 7R,15C
    const int NUM_CARS = 150;                 // number of cars
    const float ROAD_THICKNESS = 25.0f;       // thickness of the roads
    const float CLICK_TOLERANCE = 10.0f;      // determines how close your click has to be to the target to perform the action (e.g. delete road)
    const float minSpacing = 30.f;            // pixels minimum following distance for cars
    const int minspeed = 80.f;                // car min speed (default 80)
    const int maxspeed = 150.f;               // car max speed (default 150)
	const float secondsPerHour = 5.f;          // simulation speed: how many real seconds correspond to one in-sim hour

    Simulation sim(WIDTH, HEIGHT, ROWS, COLS, NUM_CARS, ROAD_THICKNESS, CLICK_TOLERANCE, minSpacing, minspeed, maxspeed, secondsPerHour);
    return sim.run();
}
