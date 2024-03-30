#include "MathC.h"

#include <numeric>
#include <valarray>

using namespace std;

bool MathC::points_within_triangle_2d(const std::vector<double> &point, const std::vector<double> &a,
                                      const std::vector<double> &b, const std::vector<double> &c,
                                      const double tolerance) {
    const double w1 = (a.at(0) * (c.at(1) - a.at(1)) + (point.at(1) - a.at(1)) * (c.at(0) - a.at(0)) - point.at(0) * (c.
            at(1) -
                                                                                                                      a.at(1))) /
                      ((b.at(1) - a.at(1)) * (c.at(0) - a.at(0)) - (b.at(0) - a.at(0)) * (c.at(1) - a.at(1)));

    const double w2 = (point.at(1) - a.at(1) - w1 * (b.at(1) - a.at(1))) /
                      (c.at(1) - a.at(1));

    return w1 >= tolerance && w2 >= tolerance && w1 + w2 <= 1.0 - tolerance;
}

bool MathC::line_intersect_2d(const std::vector<double> &start1, const std::vector<double> &end1,
                              const std::vector<double> &start2,
                              const std::vector<double> &end2) {
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

bool MathC::triangle_intersect_2d(const std::vector<double> &a1, const std::vector<double> &a2,
                                  const std::vector<double> &a3,
                                  const std::vector<double> &b1, const std::vector<double> &b2,
                                  const std::vector<double> &b3) {
    return line_intersect_2d(a1, a2, b1, b2) ||
           line_intersect_2d(a1, a3, b1, b2) ||
           line_intersect_2d(a2, a3, b1, b2) ||
           line_intersect_2d(a1, a2, b1, b3) ||
           line_intersect_2d(a1, a3, b1, b3) ||
           line_intersect_2d(a2, a3, b1, b3) ||
           line_intersect_2d(a1, a2, b2, b3) ||
           line_intersect_2d(a1, a3, b2, b3) ||
           line_intersect_2d(a2, a3, b2, b3);
}

vector<double> &MathC::closet_point_on_line(const vector<double> &point, const vector<double> &start,
                                            const vector<double> &end) {
    //Get heading
    vector<double> heading = {end.at(0) - start.at(0), end.at(1) - start.at(1)};
    const double magnitude = sqrt(pow(heading.at(0), 2) + pow(heading.at(1), 2));
    heading = {heading.at(0) / magnitude, heading.at(1) / magnitude};

    //Do projection from the point but clamp it
    vector<double> lhs = {point.at(0) - start.at(0), point.at(1) - start.at(1)};
    double dotP = inner_product(lhs.begin(), lhs.end(), heading.begin(), 0);

    dotP = dotP < 0.0 ? 0.0 : dotP > magnitude ? magnitude : dotP;

    return *new vector<double>{start.at(0) + heading.at(0) * dotP, start.at(1) + heading.at(1) * dotP};
}

std::vector<int> MathC::t_shared_between(std::vector<int> &target, std::vector<int> &other, int max) {
    std::vector<int> &result = *new std::vector<int>;

    for (const int &itemTarget: target) {
        for (const int &itemOther: other) {
            if (itemTarget == itemOther) {
                result.push_back(itemOther);
                break;
            }
        }

        if (result.size() == max)
            break;
    }

    return result;
}

bool MathC::contains(std::vector<int> &list, int element) {
    for (int t: list) {
        if (t == element)
            return true;
    }

    return false;
}

float MathC::quick_square_distance(const std::vector<double> &point1, const std::vector<double> &point2) {
    return static_cast<float>(pow(point1.at(0) - point2.at(0), 2) + pow(point1.at(1) - point2.at(1), 2));
}

float MathC::distance(const std::vector<double> &p1, const std::vector<double> &p2) {
    if (p1.size() != p2.size())
        return 0;

    double total = 0;

    for (int i = 0; i < p1.size(); ++i)
        total += pow(p1.at(i) - p2.at(i), 2);

    return static_cast<float>(sqrt(total));
}

std::vector<double> MathC::normalize(const std::vector<double> &from, const std::vector<double> &to) {
    if (from.size() != to.size())
        return {0};

    const float magnitude = distance(from, to);
    std::vector<double> result(from.size());

    for (int i = 0; i < from.size(); ++i)
        result.push_back((from.at(i) - to.at(i)) / magnitude);

    return result;
}

std::vector<double> &MathC::minus(std::vector<double> a, std::vector<double> b) {
    vector<double> &result = *new vector<double>;
    for (int i = 0; i < a.size() && i < b.size(); ++i) {
        result.push_back(a[i] - b[i]);
    }

    return result;
}

std::vector<double> &MathC::normalize(const vector<double> &vector) {
    const float magnitude = distance(vector);
    std::vector<double> &result = *new std::vector<double>;

    for (double i: vector)
        result.push_back(i / magnitude);

    return result;
}

float MathC::distance(const vector<double> &p) {
    double total = 0;

    for (double i: p)
        total += pow(i, 2);

    return static_cast<float>(sqrt(total));
}

std::vector<double> &MathC::multiply(std::vector<double> &v, float a) {
    for (double &item: v)
        item *= a;

    return v;
}

void MathC::addVectors(vector<double> &v1, vector<double> &v2) {
    for (int i = 0; i < v1.size() && i < v2.size(); ++i) {
        v1[i] += v2[i];
    }
}

std::vector<double> &MathC::lerpVectors(vector<double> &v1, vector<double> &v2, float lerp) {
    vector<double> &result = *new vector<double>;
    for (int i = 0; i < v1.size() && i < v2.size(); ++i) {
        result.push_back(v1[i] + (v2[i] - v1[i]) * lerp);
    }

    return result;
}

double &MathC::Max(double &a, double &b) {
    if (a > b)
        return a;

    return b;
}

double &MathC::Min(double &a, double &b) {
    if (a < b)
        return a;

    return b;
}
