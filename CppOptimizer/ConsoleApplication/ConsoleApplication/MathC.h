#pragma once
#include <list>
#include <vector>

class MathC
{
public:
    static bool PointsWithinTriangle2D(std::vector<double> point, std::vector<double> a, std::vector<double> b,
                                       std::vector<double> c, double tolerance = .001f);

    static bool TriangleIntersect2D(std::vector<double> a1, std::vector<double> a2, std::vector<double> a3,
                                    std::vector<double> b1, std::vector<double> b2, std::vector<double> b3);

    static bool LineIntersect2D(std::vector<double> start1, std::vector<double> end1, std::vector<double> start2,
                                std::vector<double> end2);

    static std::vector<double> ClosetPointOnLine(std::vector<double> point, std::vector<double> start,
                                                 std::vector<double> end);

    template <typename T>
    static std::list<T> TSharedBetween(std::list<T> target, std::list<T> other, int max = -1);

    static double QuickSquareDistance(std::vector<double> point1, std::vector<double> point2);
};
