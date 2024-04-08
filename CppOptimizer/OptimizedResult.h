//
// Created by Mads on 07/04/2024.
//

#ifndef CPPOPTIMIZER_OPTIMIZEDRESULT_H
#define CPPOPTIMIZER_OPTIMIZEDRESULT_H

#include <string>
#include <vector>

using namespace std;

class OptimizedResult {
public:
    explicit OptimizedResult(const int &i);

    int averageCount;
    vector<int> vertexCount, indicesCount, triangleCount;
    float totalTime;
    vector<float> individualTime;
};


#endif //CPPOPTIMIZER_OPTIMIZEDRESULT_H
