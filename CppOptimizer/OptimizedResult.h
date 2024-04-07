//
// Created by Mads on 07/04/2024.
//

#ifndef CPPOPTIMIZER_OPTIMIZEDRESULT_H
#define CPPOPTIMIZER_OPTIMIZEDRESULT_H

#include <string>

using namespace std;

class OptimizedResult {
public:
    explicit OptimizedResult(const int &i);

    int averageCount, vertexCount, indicesCount, triangleCount;
    float totalTime, averageTime;
};


#endif //CPPOPTIMIZER_OPTIMIZEDRESULT_H
