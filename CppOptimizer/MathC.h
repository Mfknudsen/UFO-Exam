#include <vector>
#include "Vector2.h"
#include "Vector3.h"

class MathC {
public:
    static bool LineIntersect2DWithTolerance(Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2);

    static bool
    PointWithinTriangle2DWithTolerance(Vector2 point, Vector2 a, Vector2 b, Vector2 c, float tolerance = .001f);

    static bool TriangleIntersect2D(Vector2 a1, Vector2 a2, Vector2 a3, Vector2 b1, Vector2 b2, Vector2 b3);

    static Vector2 &ClosetPointOnLine(Vector2 point, Vector2 start, Vector2 end);

    static float Min(float x, float x1);

    static const float Max(float y, float y1);

    static float Clamp(float p, float d, float max);

    static Vector2 &XZ(Vector3 &v);

    static Vector3 &XYZ(Vector2 v);
};