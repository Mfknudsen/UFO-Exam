#include <vector>

class MathC {
public:
    static bool points_within_triangle_2d(const std::vector<double> &point, const std::vector<double> &a,
                                          const std::vector<double> &b,
                                          const std::vector<double> &c, double tolerance = .001f);

    static bool
    triangle_intersect_2d(const std::vector<double> &a1, const std::vector<double> &a2, const std::vector<double> &a3,
                          const std::vector<double> &b1, const std::vector<double> &b2, const std::vector<double> &b3);

    static bool line_intersect_2d(const std::vector<double> &start1, const std::vector<double> &end1,
                                  const std::vector<double> &start2,
                                  const std::vector<double> &end2);

    static std::vector<double> &closet_point_on_line(const std::vector<double> &point, const std::vector<double> &start,
                                                     const std::vector<double> &end);

    static std::vector<int> t_shared_between(std::vector<int> &target, std::vector<int> &other, int max);

    static float quick_square_distance(const std::vector<double> &point1, const std::vector<double> &point2);

    static float distance(const std::vector<double> &p1, const std::vector<double> &p2);

    static float distance(const std::vector<double> &p);

    static bool contains(std::vector<int> &list, int element);

    static std::vector<double> normalize(const std::vector<double> &from, const std::vector<double> &to);

    static std::vector<double> &normalize(const std::vector<double> &vector);

    static std::vector<double> &minus(std::vector<double> a, std::vector<double> b);

    static std::vector<double> &multiply(std::vector<double> &v, float a);

    static void addVectors(std::vector<double> &v1, std::vector<double> &v2);

    static std::vector<double> &lerpVectors(std::vector<double> &v1, std::vector<double> &v2, float lerp);

    static double &Min(double &a, double &b);

    static double &Max(double &a, double &b);
};