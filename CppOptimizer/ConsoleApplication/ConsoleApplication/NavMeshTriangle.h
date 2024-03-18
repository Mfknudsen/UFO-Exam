#pragma once
#include <list>
#include <vector>

struct NavMeshTriangle
{
private:
    int id, a, b, c;
    std::vector<double> ab, bc, ca;

    std::list<int> neighborIDs;
    std::list<float> widthDistanceBetweenNeighbors;

public:
    NavMeshTriangle(int id, int a, int c, std::vector<std::vector<double>> verts);

    int ID();

    std::list<int> Vertices();

    std::list<int> Neighbors();

    void SetNeighborIDs(std::list<int> set);

    void SetBorderWidth(std::vector<std::vector<double>> verts, std::list<NavMeshTriangle> triangles);
};
