#include <cmath>
#include "NavMeshOptimized.h"

using namespace std;

vector<vector<float>> NavMeshOptimized::getVertices() {
    vector<vector<float>> result =  vector<vector<float>>();

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
NavMeshOptimized::SetValues(vector<Vector3> &vertices_in, vector<int> &indices_in, vector<NavMeshTriangle> &triangles_in, const float groupDivision) {
    vertices2D =  vector<Vector2>();
    verticesY =  vector<float>();
    indices = indices_in;

    for (Vector3 &vertex: vertices_in) {
        vertices2D.push_back( Vector2{vertex.x, vertex.z});
        verticesY.push_back(vertex.y);
    }

    triangles_ = triangles_in;

    trianglesByVertexPosition = *new map<Vector2Int, vector<int>>;

    triangleByVertexId = *new map<int, vector<int>>;

    for (int i = 0; i < verticesY.size(); ++i) {
        triangleByVertexId.insert({i,  vector<int>()});
    }

    for (NavMeshTriangle &t: triangles_) {
        for (int &vertexIndex: t.vertices()) {
            Vector2Int vertexID = {(int) floor(vertices2D[vertexIndex].x / groupDivision),
                                   (int) floor(vertices2D[vertexIndex].y / groupDivision)};

            if (trianglesByVertexPosition.find(vertexID) == trianglesByVertexPosition.end())
                trianglesByVertexPosition.insert({vertexID,  vector<int>()});

            trianglesByVertexPosition[vertexID].push_back(t.id());
        }
    }
}

vector<int> &NavMeshOptimized::getIndices() {
    return indices;
}
