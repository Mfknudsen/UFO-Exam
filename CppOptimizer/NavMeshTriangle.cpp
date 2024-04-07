#include "NavMeshTriangle.h"
#include "MathC.h"

using namespace std;

NavMeshTriangle::NavMeshTriangle(int id_in, int a_in, int b_in, int c_in) {
    id_ = id_in;
    a_ = a_in;
    b_ = b_in;
    c_ = c_in;
    neighbor_ids_ =  std::vector<int>(3, 0);
    width_distance_between_neighbors_ =  std::vector<float>(3, 0);
}

int NavMeshTriangle::id() {
    return id_;
}

vector<int> NavMeshTriangle::vertices() {
    return  {a_, b_, c_};
}

vector<int>NavMeshTriangle::neighbors() {
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

int NavMeshTriangle::GetA() {
    return a_;
}

int NavMeshTriangle::GetB() {
    return b_;
}

int NavMeshTriangle::GetC() {
    return c_;
}

void NavMeshTriangle::SetBorderWidth(const vector<Vector3> &verts, vector<NavMeshTriangle> &triangles) {

}
