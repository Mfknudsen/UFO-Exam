#include <vector>
#include <list>

using namespace std;

struct NavMeshImport {
private:
    vector<float> cleanPoint;
    vector<vector<float>> vertices;
    vector<int> indices;

public:
    NavMeshImport(const vector<float> &clean_point_in, const vector<vector<float>> &vertices_in,
                  const vector<int> &indices_in);

    vector<vector<float>> *get_vertices();

    vector<int> *get_indices();

    vector<float> *getCleanPoint();
};