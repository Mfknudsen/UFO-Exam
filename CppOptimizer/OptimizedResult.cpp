//
// Created by Mads on 07/04/2024.
//

#include "OptimizedResult.h"

OptimizedResult::OptimizedResult(const int &i) {
    averageCount = i;

    vertexCount = vector<int>();
    vertexCount.reserve(i);
    indicesCount = vector<int>();
    indicesCount.reserve(i);
    triangleCount = vector<int>();
    triangleCount.reserve(i);

    totalTime = 0;
    individualTime = vector<float>();
    individualTime.reserve(i);
}
