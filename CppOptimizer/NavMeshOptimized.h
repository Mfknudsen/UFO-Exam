#include <vector>
#include <map>
#include "NavMeshTriangle.h"

struct NavMeshOptimized {
private:
    std::vector<std::vector<double>> vertices2D;
    std::vector<double> verticesY;

    std::vector<NavMeshTriangle> triangles_;

    std::map<std::vector<double>, std::vector<int>> trianglesByVertexPosition;

    /// <summary>
    ///     Index of vertex returns all NavTriangles containing the vertex id.
    /// </summary>
    std::map<int,std::vector<int>> triangleByVertexId;
    
public:
    std::vector<std::vector<double>> get_vertices();

    std::vector<NavMeshTriangle> *get_triangles();

    void
    set_values(std::vector<std::vector<double>> &vertices_in, std::vector<NavMeshTriangle> &triangles_in, double groupDivision);
};