#include <iostream>
#include "NavMeshImport.h"

using namespace std;

NavMeshImport::NavMeshImport( vector<float> &clean_point_in,  vector<Vector3> &vertices_in,
                              vector<int> &indices_in, int finalVertexCount_in, int finalIndicesCount_in,
                             int finalTriangleCount_in) {
    cleanPoint = clean_point_in;
    vertices = vertices_in;
    indices = indices_in;

    finalVertexCount = finalVertexCount_in;
    finalIndicesCount = finalIndicesCount_in;
    finalTriangleCount = finalTriangleCount_in;
}

vector<Vector3> &NavMeshImport::getVertices() {
    return vertices;
}

vector<int> &NavMeshImport::getIndices() {
    return indices;
}

vector<float> &NavMeshImport::getCleanPoint() {
    return cleanPoint;
}

int NavMeshImport::FV() {
    return finalVertexCount;
}

int NavMeshImport::FI() {
    return finalIndicesCount;
}

int NavMeshImport::FT() {
    return finalTriangleCount;
}
