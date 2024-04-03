#include <cmath>
#include "NavMeshOptimized.h"

using namespace std;

vector<vector<float>> &NavMeshOptimized::getVertices() {
    vector<vector<float>> &result = *new vector<vector<float>>;

    for (int i = 0; i < vertices2D.size(); ++i) {
        Vector2 v = vertices2D.at(i);
        result.push_back(*new vector<float>{v.x, verticesY[i], v.y});
    }

    return result;
}

vector<NavMeshTriangle> &NavMeshOptimized::getTriangles() {
    return triangles_;
}

void
NavMeshOptimized::SetValues(vector<Vector3> &vertices_in, vector<NavMeshTriangle> &triangles_in, float groupDivision) {
    vertices2D = *new vector<Vector2>;
    verticesY = *new vector<float>;

    for (Vector3 &vertex: vertices_in) {
        vertices2D.push_back(*new Vector2{vertex.x, vertex.z});
        verticesY.push_back(vertex.y);
    }

    triangles_ = triangles_in;

    trianglesByVertexPosition = *new map<Vector2Int, vector<int>>;

    triangleByVertexId = *new map<int, vector<int>>;

    for (int i = 0; i < verticesY.size(); ++i) {
        triangleByVertexId.insert({i, *new vector<int>});
    }

    for (NavMeshTriangle &t: triangles_) {
        for (int &vertexIndex: t.vertices()) {
            Vector2Int vertexID = {(int) floor(vertices2D[vertexIndex].x / groupDivision),
                                   (int) floor(vertices2D[vertexIndex].y / groupDivision)};

            if (trianglesByVertexPosition.find(vertexID) == trianglesByVertexPosition.end())
                trianglesByVertexPosition.insert({vertexID, *new vector<int>});

            trianglesByVertexPosition[vertexID].push_back(t.id());
        }
    }
}