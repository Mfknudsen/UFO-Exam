#include <vector>
#include "Vector3.h"

using namespace std;

struct NavMeshTriangle {
private:
    int id_, a_, b_, c_;

    vector<int> neighbor_ids_;
    vector<float> width_distance_between_neighbors_;

public:
    NavMeshTriangle(int id_in, int a_in, int b_in, int c_in);

    int id();

    vector<int> vertices();

    vector<int> neighbors();

    void SetNeighborIds(const vector<int> &set);

    void SetBorderWidth(const vector<Vector3> &verts, vector<NavMeshTriangle> &triangles);

    int GetA();

    int GetB();

    int GetC();
};