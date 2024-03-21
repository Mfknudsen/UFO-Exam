#include "NavMeshOptimized.h"

std::vector<std::vector<double>> NavMeshOptimized::get_vertices()
{
    std::vector<std::vector<double>> result;

    for (int i = 0; i < vertices_.size(); ++i)
    {
        std::vector<double> v = vertices_.at(i);
        result.push_back({v.at(0), vertices_y_.at(i), v.at(1)});
    }

    return result;
}

std::vector<NavMeshTriangle>* NavMeshOptimized::get_triangles()
{
    return &triangles_;
}

void NavMeshOptimized::set_values(const std::vector<std::vector<double>>& vertices_in, const std::vector<NavMeshTriangle>& triangles_in)
{
    triangles_ = triangles_in;

    vertices_.clear();
    vertices_.reserve(vertices_in.size());
    vertices_y_.clear();
    vertices_y_.reserve(vertices_in.size());

    for (int i = 0; i < vertices_in.size(); ++i)
    {
        std::vector<double> v = vertices_in.at(i);

        vertices_.at(i) = {v.at(0), v.at(2)};
        vertices_y_.at(i) = {v.at(1)};
    }

    triangles_by_vertex_position_.clear();
    
}
