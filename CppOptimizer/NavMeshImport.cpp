#include "NavMeshImport.h"

using namespace std;

NavMeshImport::NavMeshImport(const vector<float> &clean_point_in, const vector<vector<float>> &vertices_in,
                             const vector<int> &indices_in) {
    cleanPoint = clean_point_in;
    vertices = vertices_in;
    indices = indices_in;
}

vector<vector<float>> *NavMeshImport::get_vertices() {
    return &vertices;
}

vector<int> *NavMeshImport::get_indices() {
    return &indices;
}

vector<float> *NavMeshImport::getCleanPoint() {
    return &cleanPoint;
}
