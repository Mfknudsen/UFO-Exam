#include "MathC.h"

#include <numeric>

static bool PointsWithinTriangle2D(const std::vector<double>& point, const std::vector<double>& a,
                                   const std::vector<double>& b,
                                   const std::vector<double>& c, const double tolerance = .001f)
{
    const double w1 = (a.at(0) * (c.at(1) - a.at(1)) + (point.at(1) - a.at(1)) * (c.at(0) - a.at(0)) - point.at(0) * (c.
            at(1) -
            a.at(1))) /
        ((b.at(1) - a.at(1)) * (c.at(0) - a.at(0)) - (b.at(0) - a.at(0)) * (c.at(1) - a.at(1)));

    const double w2 = (point.at(1) - a.at(1) - w1 * (b.at(1) - a.at(1))) /
        (c.at(1) - a.at(1));

    return w1 >= tolerance && w2 >= tolerance && w1 + w2 <= 1.0 - tolerance;
}

static bool LineIntersect2D(const std::vector<double>& start1, const std::vector<double>& end1,
                            const std::vector<double>& start2,
                            const std::vector<double>& end2)
{
    //Line1
    const double a1 = end1.at(1) - start1.at(1);
    const double b1 = start1.at(0) - end1.at(0);
    const double c1 = a1 * start1.at(0) + b1 * start1.at(1);

    //Line2
    const double a2 = end2.at(1) - start2.at(1);
    const double b2 = start2.at(0) - end2.at(0);
    const double c2 = a2 * start2.at(0) + b2 * start2.at(1);

    const double denominator = a1 * b2 - a2 * b1;

    if (denominator == 0.0)
        return false;

    const std::vector<double> point = {(b2 * c1 - b1 * c2) / denominator, (a1 * c2 - a2 * c1) / denominator};

    if (point == start1 || point == end1 ||
        point == start2 || point == end2)
        return false;

    constexpr double tolerance = .001f;

    return point.at(0) > std::min(start1.at(0), end1.at(0)) + tolerance &&
        point.at(0) < std::max(start1.at(0), end1.at(0)) - tolerance &&
        point.at(0) > std::min(start2.at(0), end2.at(0)) + tolerance &&
        point.at(0) < std::max(start2.at(0), end2.at(0)) - tolerance &&
        point.at(1) > std::min(start1.at(1), end1.at(1)) + tolerance &&
        point.at(1) < std::max(start1.at(1), end1.at(1)) - tolerance &&
        point.at(1) > std::min(start2.at(1), end2.at(1)) + tolerance &&
        point.at(1) < std::max(start2.at(1), end2.at(1)) - tolerance;
}

static bool TriangleIntersect2D(const std::vector<double>& a1, const std::vector<double>& a2,
                                const std::vector<double>& a3,
                                const std::vector<double>& b1, const std::vector<double>& b2,
                                const std::vector<double>& b3)
{
    return LineIntersect2D(a1, a2, b1, b2) ||
        LineIntersect2D(a1, a3, b1, b2) ||
        LineIntersect2D(a2, a3, b1, b2) ||
        LineIntersect2D(a1, a2, b1, b3) ||
        LineIntersect2D(a1, a3, b1, b3) ||
        LineIntersect2D(a2, a3, b1, b3) ||
        LineIntersect2D(a1, a2, b2, b3) ||
        LineIntersect2D(a1, a3, b2, b3) ||
        LineIntersect2D(a2, a3, b2, b3);
}

static std::vector<double> ClosetPointOnLine(const std::vector<double>& point, const std::vector<double>& start,
                                             const std::vector<double>& end)
{
    //Get heading
    std::vector<double> heading = {end.at(0) - start.at(0), end.at(1) - start.at(1)};
    const double magnitude = sqrt(pow(heading.at(0), 2) + pow(heading.at(1), 2));
    heading = {heading.at(0) / magnitude, heading.at(1) / magnitude};

    //Do projection from the point but clamp it
    std::vector<double> lhs = {point.at(0) - start.at(0), point.at(1) - start.at(1)};
    double dotP = std::inner_product(lhs.begin(), lhs.end(), heading.begin(), 0);

    dotP = dotP < 0.0 ? 0.0 : dotP > magnitude ? magnitude : dotP;

    return {start.at(0) + heading.at(0) * dotP, start.at(1) + heading.at(1) * dotP};
}

template <typename T>
static std::list<T> TSharedBetween(const std::list<T> target, const std::list<T> other, int max = -1)
{
}

static float QuickSquareDistance(const std::vector<double>& point1, const std::vector<double>& point2)
{
    return pow(point1.at(0) - point2.at(0), 2) + pow(point1.at(1) - point2.at(1), 2);
}
