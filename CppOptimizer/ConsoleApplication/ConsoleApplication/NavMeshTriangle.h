#pragma once

#include <vector>

struct NavMeshTriangle
{
private:
    int id_, a_, b_, c_;
    std::vector<double> ab_, bc_, ca_;

    std::vector<int> neighbor_ids_;
    std::vector<float> width_distance_between_neighbors_;

public:
    NavMeshTriangle(int id_in, int a_in, int b_in, int c_in, const std::vector<std::vector<double>>& verts_in);

    int id() const;

    std::vector<int> vertices();

    std::vector<int> neighbors();

    void set_neighbor_i_ds(const std::vector<int>& set);

    void set_border_width(const std::vector<std::vector<double>>& verts, std::vector<NavMeshTriangle>& triangles);
};
