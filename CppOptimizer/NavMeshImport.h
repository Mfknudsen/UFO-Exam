#include <vector>
#include <list>

struct NavMeshImport {
private:
    std::vector<double> cleanPoint;
    std::vector<std::vector<double>> vertices;
    std::vector<int> indices;

public:
    NavMeshImport(const std::vector<double> &clean_point_in, const std::vector<std::vector<double>> &vertices_in,
                  const std::vector<int> &indices_in);

    std::vector<std::vector<double>> *get_vertices();

    std::vector<int> *get_indices();

    std::vector<double> *get_clean_point();
};