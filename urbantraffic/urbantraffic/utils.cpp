#include "utils.hpp"
#include <random>

std::mt19937 rng((unsigned)std::random_device{}());

int randint(int a, int b) {
    std::uniform_int_distribution<int> d(a, b); return d(rng);
}

float randfloat(float a, float b) {
    std::uniform_real_distribution<float> d(a, b); return d(rng);
}
