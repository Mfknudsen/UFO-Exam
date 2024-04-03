//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#ifndef CPPOPTIMIZER_VECTOR3_H
#define CPPOPTIMIZER_VECTOR3_H


#include "Vector2.h"

class Vector3 {
public:
    float x, y, z;

    [[nodiscard]] Vector2 XZ() const;

    float QuickSquareDistance(const Vector3 &other);

    Vector3 operator-(const Vector3 &b) {
        x -= b.x;
        y -= b.y;
        z -= b.z;
        return *this;
    };

    [[nodiscard]] float SqrMagnitude() const;
};


#endif //CPPOPTIMIZER_VECTOR3_H
