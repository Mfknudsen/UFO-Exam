#pragma once
#include <list>
#include <map>
#include <vector>

#include "NavMeshTriangle.h"

struct NavMeshOptimized
{
private:
    std::vector<std::vector<double>> vertices;
    std::list<double> verticesY;

    std::list<NavMeshTriangle> triangles;

    std::map<std::vector<double>, std::list<int>> trianglesByVertexPosition;

    /// <summary>
    ///     Index of vertex returns all NavTriangles containing the vertex id.
    /// </summary>
    std::list<std::list<int>> triangleByVertexId;

    const double GroupDivision = 10.0;

public:
    std::vector<std::vector<double>> GetVertices();

    std::list<NavMeshTriangle> GetTriangles();

    void SetValues(std::vector<std::vector<double>> vertices, std::list<NavMeshTriangle> navTriangles);
};
