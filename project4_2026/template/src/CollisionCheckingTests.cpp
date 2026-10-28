#include "CollisionChecking.h"
#include <cmath>
#include <iostream>
#include <stdexcept>

int main()
{
    const std::vector<Rectangle> obstacles{{1, 1, 2, 2}};
    auto check = [](bool condition) { if (!condition) throw std::runtime_error("Collision test failed"); };
    check(!isValidPoint(2, 2, obstacles));
    check(!isValidPoint(1, 2, obstacles));
    check(isValidPoint(0, 2, obstacles));
    check(isValidPoint(4, 4, obstacles));
    check(isValidPoint(2, 2, {}));
    check(!isValidCircle(2, 2, 0.1, obstacles));
    check(!isValidCircle(0, 2, 1, obstacles)); // tangency
    check(!isValidCircle(2, 2, 0, obstacles));
    check(isValidCircle(0, 0, 0.5, obstacles));
    check(!isValidSquare(2, 2, 0, 0.5, obstacles)); // square contained
    check(!isValidSquare(2, 2, 0, 6, obstacles)); // obstacle contained
    check(!isValidSquare(0.5, 2, 0, 1, obstacles)); // edge contact
    check(!isValidSquare(0.8, 2, std::acos(-1.0) / 4, 1, obstacles));
    check(isValidSquare(-1, -1, 0.7, 1, obstacles));
    std::cout << "Collision tests passed: inside/outside, boundaries, circle tangency, rotated/contained squares.\n";
}
