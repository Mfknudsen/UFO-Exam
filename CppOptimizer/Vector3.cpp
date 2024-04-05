//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#include <cmath>
#include <iostream>
#include "Vector3.h"

using namespace std;

float Vector3::QuickSquareDistance(Vector3 &other) {
    return (*this - other).SqrMagnitude();
}

float Vector3::SqrMagnitude() const {
    cout << (float)x * (float)x << "\n";
    return x * x + y * y + z * z;
}

float Vector3::Distance(const Vector3 &a, const Vector3 &b) {
    float x = a.x - b.x;
    float y = a.y - b.y;
    float z = a.z - b.z;
    return sqrt(x * x + y * y + z * z);
}

Vector3::Vector3(float xIn, float yIn, float zIn) {
    x = xIn;
    y = yIn;
    z = zIn;
}


float Vector3::Magnitude() const {
    return sqrt(x * x + y * y + z * z);
}

Vector3 &Vector3::Normalize() const {
    float m = Magnitude();
    return *new Vector3(x / m, y / m, z / m);
}

Vector3::Vector3() {
    x = 0;
    y = 0;
    z = 0;
}
