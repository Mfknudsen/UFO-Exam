//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#include "Vector3.h"

Vector2 Vector3::XZ() const {
    return {x, z};
}

float Vector3::QuickSquareDistance(const Vector3 &other) {
    return (*this - other).SqrMagnitude();
}

float Vector3::SqrMagnitude() const {
    return x * x + y * y + z * z;
}
