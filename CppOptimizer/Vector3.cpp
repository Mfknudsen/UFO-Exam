//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#include <cmath>
#include "Vector3.h"

using namespace std;

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


Vector3::Vector3() {
    x = 0;
    y = 0;
    z = 0;
}
