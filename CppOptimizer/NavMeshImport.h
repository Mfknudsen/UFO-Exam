#include <vector>
#include <list>
#include "Vector3.h"

using namespace std;

struct NavMeshImport {
private:
    vector<float> cleanPoint;
    vector<Vector3> vertices;
    vector<int> indices;
    int finalVertexCount, finalTriangleCount, finalIndicesCount;

public:
    NavMeshImport(vector<float> &clean_point_in, vector<Vector3> &vertices_in,
                  vector<int> &indices_in, int finalVertexCount_in, int finalIndicesCount_in,
                  int finalTriangleCount_in);

    vector<Vector3> &getVertices();

    vector<int> &getIndices();

    vector<float> &getCleanPoint();

    int FV();

    int FI();

    int FT();
};