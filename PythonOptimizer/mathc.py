from vectors import vector2


class MathC:
    @staticmethod
    def line_intersect_2d_with_tolerance(start1, end1, start2, end2, tolerance=0.001):
        a1 = end1.y - start1.y
        b1 = start1.x - end1.x
        c1 = a1 * start1.x + b1 * start1.y


        a2 = end2.y - start2.y
        b2 = start2.x - end2.x
        c2 = a2 * start2.x + b2 * start2.y

        denominator = a1 * b2 - a2 * b1

        if denominator == 0:
            print("no denominator")
            return False

        point = vector2((b2 * c1 - b1 * c2) / denominator, (a1 * c2 - a2 * c1) / denominator)

        if point == start1 or point == end1 or point == start2 or point == end2:
            return False

        return (min(start1.x, end1.x) + tolerance < point.x < max(start1.x, end1.x) - tolerance and
                min(start2.x, end2.x) + tolerance < point.x < max(start2.x, end2.x) - tolerance and
                min(start1.y, end1.y) + tolerance < point.y < max(start1.y, end1.y) - tolerance and
                min(start2.y, end2.y) + tolerance < point.y < max(start2.y, end2.y) - tolerance)

    @staticmethod
    def point_within_triangle_2d_with_tolerance(point, a, b, c, tolerance=0.001):
        s1 = c.y - a.y + 0.0001
        s2 = c.x - a.x
        s3 = b.y - a.y
        s4 = point.y - a.y

        w1 = (a.x * s1 + s4 * s2 - point.x * s1) / (s3 * s2 - (b.x - a.x + 0.0001) * s1)
        w2 = (s4 - w1 * s3) / s1

        return tolerance < w1 < 1 - tolerance and tolerance < w2 < 1 - tolerance and w1 + w2 < 1 - tolerance

    @staticmethod
    def triangle_intersect_2d(a1, a2, a3, b1, b2, b3):
        methods = MathC.line_intersect_2d_with_tolerance
        return any(methods(pair[0], pair[1], pair[2], pair[3]) for pair in [
            (a1, a2, b1, b2), (a1, a3, b1, b2), (a2, a3, b1, b2),
            (a1, a2, b1, b3), (a1, a3, b1, b3), (a2, a3, b1, b3),
            (a1, a2, b2, b3), (a1, a3, b2, b3), (a2, a3, b2, b3)
        ])
