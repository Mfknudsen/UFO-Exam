#include <vector>
#include <list>

using namespace std;

struct NavMeshImport {
private:
    vector<float> cleanPoint;
    vector<vector<float>> vertices;
    vector<int> indices;
    int finalVertexCount, finalTriangleCount, finalIndicesCount;

public:
    NavMeshImport(const vector<float> &clean_point_in, const vector<vector<float>> &vertices_in,
                  const vector<int> &indices_in, int finalVertexCount_in, int finalIndicesCount_in,
                  int finalTriangleCount_in);

    vector<vector<float>> *getVertices();

    vector<int> *getIndices();

    vector<float> *getCleanPoint();

    int FV();

    int FI();

    int FT();
};