#include <vector>

using namespace std;

struct NavMeshTriangle {
private:
    int id_, a_, b_, c_;
    vector<double> ab_, bc_, ca_;

    vector<int> neighbor_ids_;
    vector<float> width_distance_between_neighbors_;

public:
    NavMeshTriangle(int id_in, int a_in, int b_in, int c_in, vector<vector<double>> &verts_in);

    int id();

    vector<int> &vertices();

    vector<int> &neighbors();

    void SetNeighborIds(const vector<int> &set);

    void set_border_width(const vector<vector<double>> &verts, vector<NavMeshTriangle> &triangles);

    int GetA();

    int GetB();

    int GetC();
};