//
// Created by Mads T.F. Knudsen on 03/04/2024.
//

#ifndef CPPOPTIMIZER_VECTOR2INT_H
#define CPPOPTIMIZER_VECTOR2INT_H


class Vector2Int {
public:
    Vector2Int(int xIn, int yIn);

    int x, y;

    bool operator<(const Vector2Int &other) const {
        return x < other.x || (x == other.x && y < other.y);    }
};


#endif //CPPOPTIMIZER_VECTOR2INT_H
