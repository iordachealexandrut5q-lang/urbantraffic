#pragma once
#include <random>

// for random
extern std::mt19937 rng;
int randint(int a, int b);
float randfloat(float a, float b);

// integer key
inline long long edgeKey(int a, int b) {
    return ((long long)a << 32) | (unsigned int)b;
}
