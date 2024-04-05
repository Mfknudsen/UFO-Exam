#include "MathC.h"

using namespace std;

bool MathC::LineIntersect2DWithTolerance(Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2) {
    //Line1
    float a1 = end1.y - start1.y;
    float b1 = start1.x - end1.x;
    float c1 = a1 * start1.x + b1 * start1.y;

    //Line2
    float a2 = end2.y - start2.y;
    float b2 = start2.x - end2.x;
    float c2 = a2 * start2.x + b2 * start2.y;

    float denominator = a1 * b2 - a2 * b1;

    if (denominator == 0)
        return false;

    Vector2 point = *new Vector2((b2 * c1 - b1 * c2) / denominator, (a1 * c2 - a2 * c1) / denominator);

    if (point == start1 || point == end1 ||
        point == start2 || point == end2)
        return false;

    const float tolerance = .001f;

    return point.x > MathC::Min(start1.x, end1.x) + tolerance &&
           point.x < MathC::Max(start1.x, end1.x) - tolerance &&
           point.x > MathC::Min(start2.x, end2.x) + tolerance &&
           point.x < MathC::Max(start2.x, end2.x) - tolerance &&
           point.y > MathC::Min(start1.y, end1.y) + tolerance &&
           point.y < MathC::Max(start1.y, end1.y) - tolerance &&
           point.y > MathC::Min(start2.y, end2.y) + tolerance &&
           point.y < MathC::Max(start2.y, end2.y) - tolerance;
}

bool MathC::PointWithinTriangle2DWithTolerance(Vector2 point, Vector2 a, Vector2 b, Vector2 c, float tolerance) {
    float s1 = c.y - a.y + 0.0001f;
    float s2 = c.x - a.x;
    float s3 = b.y - a.y;
    float s4 = point.y - a.y;

    float w1 = (a.x * s1 + s4 * s2 - point.x * s1) / (s3 * s2 - (b.x - a.x + 0.0001f) * s1);
    float w2 = (s4 - w1 * s3) / s1;
    return w1 >= tolerance && w2 >= tolerance && w1 + w2 <= 1.0f - tolerance;
}

bool MathC::TriangleIntersect2D(Vector2 a1, Vector2 a2, Vector2 a3, Vector2 b1, Vector2 b2, Vector2 b3) {
    return LineIntersect2DWithTolerance(a1, a2, b1, b2) ||
           LineIntersect2DWithTolerance(a1, a3, b1, b2) ||
           LineIntersect2DWithTolerance(a2, a3, b1, b2) ||
           LineIntersect2DWithTolerance(a1, a2, b1, b3) ||
           LineIntersect2DWithTolerance(a1, a3, b1, b3) ||
           LineIntersect2DWithTolerance(a2, a3, b1, b3) ||
           LineIntersect2DWithTolerance(a1, a2, b2, b3) ||
           LineIntersect2DWithTolerance(a1, a3, b2, b3) ||
           LineIntersect2DWithTolerance(a2, a3, b2, b3);
}

Vector2 &MathC::ClosetPointOnLine(Vector2 point, Vector2 start, Vector2 end) {
    //Get heading
    Vector2 heading = end - start;
    float magnitudeMax = heading.Magnitude();
    heading.Normalize();

    //Do projection from the point but clamp it
    Vector2 lhs = point - start;
    float dotP = Vector2::Dot(lhs, heading);
    dotP = MathC::Clamp(dotP, 0.0f, magnitudeMax);

    Vector2 &result = *new Vector2(start.x + heading.x * dotP, start.y + heading.y * dotP);
    return result;
}

float MathC::Min(float x, float x1) {
    if (x1 < x)
        return x1;

    return x;
}

const float MathC::Max(float y, float y1) {
    if (y1 < y)
        return y1;

    return y;
}

float MathC::Clamp(float p, float d, float max) {
    return 0;
}

Vector3 &MathC::XYZ(Vector2 v) {
    return *new Vector3(v.x, 0, v.y);
}

Vector2 &MathC::XZ(Vector3 &v) {
    return *new Vector2(v.x, v.z);
}
