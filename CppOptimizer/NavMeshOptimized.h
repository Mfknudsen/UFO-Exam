#include <vector>
#include <map>
#include "NavMeshTriangle.h"

using namespace std;

struct NavMeshOptimized {
private:
    vector<std::vector<float>> vertices2D;
    vector<float> verticesY;

    vector<NavMeshTriangle> triangles_;

    map<vector<float>, vector<int>> trianglesByVertexPosition;

    /// <summary>
    ///     Index of vertex returns all NavTriangles containing the vertex id.
    /// </summary>
    map<int,vector<int>> triangleByVertexId;

public:
    vector<vector<float>> &getVertices();

    vector<NavMeshTriangle> &getTriangles();

    void
    setValues(vector<vector<float>> &vertices_in, vector<NavMeshTriangle> &triangles_in, float groupDivision);
};