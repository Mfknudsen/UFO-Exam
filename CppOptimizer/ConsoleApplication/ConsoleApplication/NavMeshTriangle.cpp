#include "NavMeshTriangle.h"

#include "MathC.h"

NavMeshTriangle::NavMeshTriangle(const int id_in, const int a_in, const int b_in, const int c_in,
                                 const std::vector<std::vector<double>>& verts_in)
{
    id_ = id_in;
    a_ = a_in;
    b_ = b_in;
    c_ = c_in;

    ab_ = MathC::normalize(verts_in.at(0), verts_in.at(1));
    bc_ = MathC::normalize(verts_in.at(1), verts_in.at(2));
    ca_ = MathC::normalize(verts_in.at(2), verts_in.at(0));
}

int NavMeshTriangle::id() const
{
    return id_;
}

std::vector<int> NavMeshTriangle::vertices()
{
    return {a_, b_, c_};
}

std::vector<int> NavMeshTriangle::neighbors()
{
    return neighbor_ids_;
}

void NavMeshTriangle::set_neighbor_i_ds(const std::vector<int>& set)
{
    neighbor_ids_.clear();
    for (int element : set)
    {
        bool exist = false;

        for (const int n : neighbor_ids_)
        {
            if (element != n)
                continue;

            exist = true;
            break;
        }

        if (!exist)
            neighbor_ids_.push_back(element);
    }
}

void NavMeshTriangle::set_border_width(const std::vector<std::vector<double>>& verts,
                                       std::vector<NavMeshTriangle>& triangles)
{
    if (neighbor_ids_.size() == 0.0)
        return;

    width_distance_between_neighbors_.clear();
    for (int i = 0; i < neighbor_ids_.size(); ++i)
        width_distance_between_neighbors_.push_back(0);

    for (int i = 0; i < neighbor_ids_.size(); ++i)
    {
        const int other_id = neighbor_ids_.at(i);
        NavMeshTriangle other = triangles.at(other_id);
        std::vector<int> ids = MathC::t_shared_between(other.vertices(), vertices());

        if (ids.size() != 2)
            continue;

        float dist = MathC::distance(verts.at(ids.at(0)), verts.at(ids.at(1)));

        if (i + 2 < neighbor_ids_.size())
        {
            int connected_border_neighbor = -1;

            if (MathC::contains(other.neighbor_ids_, neighbor_ids_.at(i + 2)))
                connected_border_neighbor = i + 2;
            else if (MathC::contains(other.neighbors(), neighbor_ids_.at(i + 1)))
                connected_border_neighbor = i + 1;

            if (connected_border_neighbor > -1)
            {
                ids = MathC::t_shared_between(
                    triangles.at(neighbor_ids_.at(connected_border_neighbor)).vertices(),
                    vertices());

                if (ids.size() == 2)
                {
                    dist += MathC::distance(verts.at(ids.at(0)), verts.at(ids.at(1)));
                    width_distance_between_neighbors_.at(connected_border_neighbor) = dist;
                }
            }
        }
        else if (i + 1 < neighbor_ids_.size())
        {
            if (MathC::contains(other.neighbor_ids_, neighbor_ids_.at(i + 1)))
            {
                ids = MathC::t_shared_between(triangles.at(neighbor_ids_.at(i + 1)).vertices(), vertices());

                if (ids.size() == 2)
                {
                    dist += MathC::distance(verts.at(ids.at(0)), verts.at(ids.at(1)));
                    width_distance_between_neighbors_.at(i + 1) = dist;
                }
            }
        }

        width_distance_between_neighbors_.at(i) = dist;
    }
}
