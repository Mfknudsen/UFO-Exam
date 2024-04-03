//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#ifndef CPPOPTIMIZER_VECTOR2_H
#define CPPOPTIMIZER_VECTOR2_H


class Vector2 {

public:
    float x, y;

    Vector2 operator-(const Vector2 &b) {
        x -= b.x;
        y -= b.y;
        return *this;
    };

    Vector2 operator+(const Vector2 &b) {
        x += b.x;
        y += b.y;
        return *this;
    };

    Vector2 operator*(const float m) {
        x *= m;
        y *= m;
        return *this;
    }

    bool operator==(const Vector2 &other) const {
        return x == other.x && y == other.y;
    }

    Vector2(float xC, float yC);

    void Normalize();

    float Magnitude();

    static float Dot(Vector2 a, Vector2 b);

    static float Distance(Vector2 vector1, Vector2 vector2);
};


#endif //CPPOPTIMIZER_VECTOR2_H
