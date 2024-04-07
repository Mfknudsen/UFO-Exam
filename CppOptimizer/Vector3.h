//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#ifndef CPPOPTIMIZER_VECTOR3_H
#define CPPOPTIMIZER_VECTOR3_H


#include "Vector2.h"

class Vector3 {
public:
    float x, y, z;

    Vector3(float xIn, float yIn, float zIn);

    Vector3();
    
    Vector3 operator-(const Vector3 &b) {
        x = x - b.x;
        y = y - b.y;
        z = z - b.z;
        return *this;
    };

    Vector3 operator+(const Vector3 &b) {
        x = b.x;
        y = b.y;
        z = b.z;
        return *this;
    };

    bool operator==(const Vector3 &b) const {
        return x == b.x && y == b.y && z == b.z;
    };

    static float Distance(const Vector3 &a, const Vector3 &b);
};


#endif //CPPOPTIMIZER_VECTOR3_H
