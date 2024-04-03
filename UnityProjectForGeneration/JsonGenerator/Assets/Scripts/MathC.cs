#region Libraries

using System;
using UnityEngine;

#endregion

public static class MathC
{
    public static bool LineIntersect2DWithTolerance(Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2)
    {
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

        Vector2 point = new Vector2((b2 * c1 - b1 * c2) / denominator, (a1 * c2 - a2 * c1) / denominator);

        if (point == start1 || point == end1 ||
            point == start2 || point == end2)
            return false;

        const float tolerance = .001f;

        return point.x > MathF.Min(start1.x, end1.x) + tolerance &&
               point.x < MathF.Max(start1.x, end1.x) - tolerance &&
               point.x > MathF.Min(start2.x, end2.x) + tolerance &&
               point.x < MathF.Max(start2.x, end2.x) - tolerance &&
               point.y > MathF.Min(start1.y, end1.y) + tolerance &&
               point.y < MathF.Max(start1.y, end1.y) - tolerance &&
               point.y > MathF.Min(start2.y, end2.y) + tolerance &&
               point.y < MathF.Max(start2.y, end2.y) - tolerance;
    }

    public static bool PointWithinTriangle2DWithTolerance(Vector2 point, Vector2 a, Vector2 b, Vector2 c,
        float tolerance = .001f)
    {
        float s1 = c.y - a.y + 0.0001f;
        float s2 = c.x - a.x;
        float s3 = b.y - a.y;
        float s4 = point.y - a.y;

        float w1 = (a.x * s1 + s4 * s2 - point.x * s1) / (s3 * s2 - (b.x - a.x + 0.0001f) * s1);
        float w2 = (s4 - w1 * s3) / s1;
        return w1 >= tolerance && w2 >= tolerance && w1 + w2 <= 1f - tolerance;
    }
    
    public static bool TriangleIntersect2D(Vector2 a1, Vector2 a2, Vector2 a3, Vector2 b1, Vector2 b2, Vector2 b3)
    {
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

    public static Vector2 ClosetPointOnLine(Vector2 point, Vector2 start, Vector2 end)
    {
        //Get heading
        Vector2 heading = end - start;
        float magnitudeMax = heading.magnitude;
        heading.Normalize();

        //Do projection from the point but clamp it
        Vector2 lhs = point - start;
        float dotP = Vector2.Dot(lhs, heading);
        dotP = Mathf.Clamp(dotP, 0f, magnitudeMax);

        return start + heading * dotP;
    }
}