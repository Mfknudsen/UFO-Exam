#include "NavMeshTriangle.h"
#include "MathC.h"

using namespace std;

NavMeshTriangle::NavMeshTriangle(int id_in, int a_in, int b_in, int c_in,
                                 vector<vector<float>> *verts_in) {
    id_ = id_in;
    a_ = a_in;
    b_ = b_in;
    c_ = c_in;

    ab_ = MathC::normalize(verts_in->at(0), verts_in->at(1));
    bc_ = MathC::normalize(verts_in->at(1), verts_in->at(2));
    ca_ = MathC::normalize(verts_in->at(2), verts_in->at(0));

    neighbor_ids_ = *new std::vector<int>(3, 0);
    width_distance_between_neighbors_ = *new std::vector<float>(3, 0);
}

int NavMeshTriangle::id() {
    return id_;
}

vector<int> &NavMeshTriangle::vertices() {
    return *new std::vector<int>{a_, b_, c_};
}

vector<int> &NavMeshTriangle::neighbors() {
    return neighbor_ids_;
}

void NavMeshTriangle::SetNeighborIds(const vector<int> &set) {
    neighbor_ids_.clear();
    for (const int &element: set) {
        bool exist = false;

        for (const int n: neighbor_ids_) {
            if (element != n)
                continue;

            exist = true;
            break;
        }

        if (!exist)
            neighbor_ids_.push_back(element);
    }
}

void NavMeshTriangle::setBorderWidth(const vector<vector<float>> &verts,
                                     vector<NavMeshTriangle> &triangles) {
    if (neighbor_ids_.empty())
        return;

    width_distance_between_neighbors_.clear();
    for (int i = 0; i < neighbor_ids_.size(); ++i)
        width_distance_between_neighbors_.push_back(0);

    for (int i = 0; i < neighbor_ids_.size(); ++i) {
        const int other_id = neighbor_ids_[i];
        NavMeshTriangle other = triangles[other_id];
        std::vector<int> ids = MathC::t_shared_between(other.vertices(), vertices(), 2);

        if (ids.size() != 2)
            continue;

        float dist = MathC::distance(&verts[ids[0]], &verts[ids[1]]);

        if (i + 2 < neighbor_ids_.size()) {
            int connected_border_neighbor = -1;

            if (MathC::contains(other.neighbor_ids_, neighbor_ids_[i + 2]))
                connected_border_neighbor = i + 2;
            else if (MathC::contains(other.neighbor_ids_, neighbor_ids_[i + 1]))
                connected_border_neighbor = i + 1;

            if (connected_border_neighbor > -1) {
                ids = MathC::t_shared_between(triangles.at(neighbor_ids_[connected_border_neighbor]).vertices(),
                                              vertices(), 2);

                if (ids.size() == 2) {
                    dist += MathC::distance(&verts[ids[0]], &verts[ids[1]]);
                    width_distance_between_neighbors_[connected_border_neighbor] = dist;
                }
            }
        } else if (i + 1 < neighbor_ids_.size()) {
            if (MathC::contains(other.neighbor_ids_, neighbor_ids_[i + 1])) {
                ids = MathC::t_shared_between(triangles.at(neighbor_ids_[i + 1]).vertices(), vertices(), 2);

                if (ids.size() == 2) {
                    dist += MathC::distance(&verts[ids[0]], &verts[ids[1]]);
                    width_distance_between_neighbors_[i + 1] = dist;
                }
            }
        }

        width_distance_between_neighbors_.at(i) = dist;
    }
}

int NavMeshTriangle::GetA() {
    return a_;
}

int NavMeshTriangle::GetB() {
    return b_;
}

int NavMeshTriangle::GetC() {
    return c_;
}
