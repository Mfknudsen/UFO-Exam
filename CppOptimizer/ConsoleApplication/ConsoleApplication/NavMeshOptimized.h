#pragma once
#include <list>
#include <map>
#include <vector>

#include "NavMeshTriangle.h"

struct NavMeshOptimized
{
private:
    std::vector<std::vector<double>> vertices_;
    std::vector<double> vertices_y_;

    std::vector<NavMeshTriangle> triangles_;

    std::map<std::vector<double>, std::vector<int>> triangles_by_vertex_position_;

    /// <summary>
    ///     Index of vertex returns all NavTriangles containing the vertex id.
    /// </summary>
    std::vector<std::vector<int>> triangle_by_vertex_id_;

    const double GroupDivision = 10.0;

public:
    std::vector<std::vector<double>> get_vertices();

    std::vector<NavMeshTriangle>* get_triangles();

    void set_values(const std::vector<std::vector<double>>& vertices_in, const std::vector<NavMeshTriangle>& triangles_in);
};
