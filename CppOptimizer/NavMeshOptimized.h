#include <vector>
#include <map>
#include "NavMeshTriangle.h"
#include "Vector3.h"
#include "Vector2Int.h"

using namespace std;

struct NavMeshOptimized {
private:
    vector<Vector2> vertices2D;
    vector<float> verticesY;
    vector<int> indices;

    vector<NavMeshTriangle> triangles_;

    map<Vector2Int, vector<int>> trianglesByVertexPosition;

    /// <summary>
    ///     Index of vertex returns all NavTriangles containing the vertex id.
    /// </summary>
    map<int, vector<int>> triangleByVertexId;

public:
    vector<vector<float>> &getVertices();

    vector<int> &getIndices();

    vector<NavMeshTriangle> &getTriangles();

    void
    SetValues(vector<Vector3> *vertices_in,vector<int> *indices_in, vector<NavMeshTriangle> *triangles_in, float groupDivision);
};