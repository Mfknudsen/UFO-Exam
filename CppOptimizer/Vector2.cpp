//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#include <cmath>
#include "Vector2.h"

using namespace std;

void Vector2::Normalize() {
    float m = Magnitude();
    if (m == 0)
        return;

    x = x / m;
    y = y / m;
}

float Vector2::Magnitude() {
    return 0;
}

Vector2::Vector2(float xC, float yC) {
    x = xC;
    y = yC;
}

float Vector2::Dot(Vector2 a, Vector2 b) {
    return 0;
}

float Vector2::Distance(Vector2 vector1, Vector2 vector2) {
    return sqrt(vector1.x * vector2.x + vector1.y * vector2.y);
}
