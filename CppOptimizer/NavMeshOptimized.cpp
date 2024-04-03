#include <cmath>
#include "NavMeshOptimized.h"

using namespace std;

vector<vector<float>> &NavMeshOptimized::getVertices() {
    vector<vector<float>> &result = *new vector<vector<float>>;

    for (int i = 0; i < vertices2D.size(); ++i) {
        vector<float> v = vertices2D.at(i);
        result.push_back(*new vector<float>{v[0], verticesY[i], v[1]});
    }

    return result;
}

vector<NavMeshTriangle> &NavMeshOptimized::getTriangles() {
    return triangles_;
}

void NavMeshOptimized::setValues(vector<vector<float>> &vertices_in,
                                 vector<NavMeshTriangle> &triangles_in, float groupDivision) {
    vertices2D = *new vector<vector<float>>;
    verticesY = *new vector<float>;

    for (vector<float> &vertex: vertices_in) {
        vertices2D.push_back({vertex[0], vertex[2]});
        verticesY.push_back(vertex[1]);
    }

    triangles_ = triangles_in;

    trianglesByVertexPosition = *new map<vector<float>, vector<int>>;

    triangleByVertexId = *new map<int, vector<int>>;

    for (int i = 0; i < verticesY.size(); ++i) {
        triangleByVertexId.insert({i, *new vector<int>});
    }

    for (NavMeshTriangle &t: triangles_) {
        for (int &vertexIndex: t.vertices()) {
            vector<float> vertexID = {floor(vertices2D[vertexIndex][0] / groupDivision),
                                       floor(vertices2D[vertexIndex][1] / groupDivision)};

            if (trianglesByVertexPosition.find(vertexID) == trianglesByVertexPosition.end())
                trianglesByVertexPosition.insert({vertexID, *new vector<int>});

            trianglesByVertexPosition[vertexID].push_back(t.id());
        }
    }
}