#region Libraries

using System.Collections.Generic;
using UnityEngine;

#endregion

public static class Extensions
{
    public static float QuickSquareDistance(this Vector3 point1, Vector3 point2) => (point1 - point2).sqrMagnitude;

    public static float Squared(this float current) => current * current;

    public static Vector2 XZ(this Vector3 target) => new Vector2(target.x, target.z);

    public static Vector3 ToV3(this Vector2 t, float y) => new Vector3(t.x, y, t.y);

    public static T[] SharedBetween<T>(this T[] target, T[] other, int max = -1)
    {
        List<T> result = new List<T>();

        foreach (T a in target)
        {
            foreach (T b in other)
            {
                if (a.Equals(b))
                {
                    result.Add(a);
                    break;
                }

                if (result.Count == max)
                    break;
            }

            if (result.Count == max)
                break;
        }

        return result.ToArray();
    }
}