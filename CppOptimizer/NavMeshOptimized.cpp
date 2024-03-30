#include <cmath>
#include "NavMeshOptimized.h"

std::vector<std::vector<double>> NavMeshOptimized::get_vertices() {
    std::vector<std::vector<double>> result;

    for (int i = 0; i < vertices2D.size(); ++i) {
        std::vector<double> v = vertices2D.at(i);
        result.push_back({v.at(0), verticesY.at(i), v.at(1)});
    }

    return result;
}

std::vector<NavMeshTriangle> *NavMeshOptimized::get_triangles() {
    return &triangles_;
}

void NavMeshOptimized::set_values(std::vector<std::vector<double>> &vertices_in,
                                  std::vector<NavMeshTriangle> &triangles_in, const double groupDivision) {
    vertices2D = *new vector<vector<double>>;
    verticesY = *new vector<double>;

    for (vector<double> &vertex: vertices_in) {
        vertices2D.push_back({vertex[0], vertex[2]});
        verticesY.push_back(vertex[1]);
    }

    triangles_ = triangles_in;

    trianglesByVertexPosition = *new map<vector<double>, vector<int>>;

    triangleByVertexId = *new map<int, vector<int>>;

    for (int i = 0; i < verticesY.size(); ++i) {
        triangleByVertexId.insert({i, *new vector<int>});
    }

    for (NavMeshTriangle &t: triangles_) {
        for (int &vertexIndex: t.vertices()) {
            vector<double> vertexID = {floor(vertices2D[vertexIndex][0] / groupDivision),
                                       floor(vertices2D[vertexIndex][1] / groupDivision)};

            if (trianglesByVertexPosition.find(vertexID) == trianglesByVertexPosition.end())
                trianglesByVertexPosition.insert({vertexID, *new vector<int>});

            trianglesByVertexPosition[vertexID].push_back(t.id());
        }
    }
}