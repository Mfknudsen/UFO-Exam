#pragma once
#include <list>
#include <vector>

struct NavMeshImport
{
private:
    std::vector<double> cleanPoint;
    std::vector<std::vector<double>> vertices;
    std::list<int> indices;

public:
    NavMeshImport(std::vector<double> cleanPoint, std::vector<std::vector<double>> vertices, std::list<int> indices);

    std::vector<std::vector<double>> GetVertices();

    std::list<int> GetIndices();

    std::vector<double> GetCleanPoint();
};
