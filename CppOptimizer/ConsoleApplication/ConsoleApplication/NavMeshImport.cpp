#include "NavMeshImport.h"

NavMeshImport::NavMeshImport(const std::vector<double>& clean_point_in, const std::vector<std::vector<double>>& vertices_in,
                             const std::list<int>& indices_in)
{
    cleanPoint = clean_point_in;
    vertices = vertices_in;
    indices = indices_in;
}

std::vector<std::vector<double>>* NavMeshImport::get_vertices()
{
    return &vertices;
}

std::list<int>* NavMeshImport::get_indices()
{
    return &indices;
}

std::vector<double>* NavMeshImport::get_clean_point()
{
    return &cleanPoint;
}
