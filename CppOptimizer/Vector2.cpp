//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#include <cmath>
#include <iostream>
#include "Vector2.h"

using namespace std;

float Vector2::Magnitude() const {
    return sqrt(x * x + y * y);
}

Vector2::Vector2(float xC, float yC) {
    x = xC;
    y = yC;
}

float Vector2::Dot(Vector2 &a, Vector2 &b) {
    return (a.x * b.x) + (a.y * b.y);
}

float Vector2::Distance(Vector2 &vector1, Vector2 &vector2) {
    return sqrt(vector1.x * vector2.x + vector1.y * vector2.y);
}

Vector2 Vector2::Lerp(Vector2 a, Vector2 b, float d) {
    return a + (b - a) * d;
}

void Vector2::NormalizeSelf() {
    float m = Magnitude();

    if (m == 0)
        return;

    x = x / m;
    y = y / m;
}


