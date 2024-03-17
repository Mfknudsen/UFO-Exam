// See https://aka.ms/new-console-template for more information

#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type.

using System.Diagnostics;
using System.Numerics;
using System.Text.Json;

// ReSharper disable CollectionNeverQueried.Local
// ReSharper disable NotAccessedField.Local
// ReSharper disable RedundantExplicitArrayCreation

namespace ConsoleApp1;

internal static class ProgramMain
{
    public static void Main()
    {
        NavMeshImport navMeshImport = LoadJsonToNavMeshImport("file");
        Stopwatch stopwatch = Stopwatch.StartNew();
        NavMeshOptimized navMeshOptimized = OptimizeNavMesh(navMeshImport);
        Console.WriteLine($"Time: {stopwatch.ElapsedMilliseconds}");
        Console.WriteLine(
            $"Vertex count: Import({navMeshImport.GetVertices().Count})  |  Optimized({navMeshOptimized.GetVertices().Length})");
        Console.WriteLine(
            $"Triangle count: Import({navMeshImport.GetIndices().Count / 3})  |  Optimized({navMeshOptimized.GetTriangles().Length})");
    }

    /// <summary>
    ///     Load data generated in Unity and stored into a Json file.
    /// </summary>
    /// <returns>Import struct containing relevant information of the navigation mesh</returns>
    private static NavMeshImport LoadJsonToNavMeshImport(string fileName)
    {
        using StreamReader r = new StreamReader($"{fileName}.json");

        string jsonString = r.ReadToEnd();
        return JsonSerializer.Deserialize<NavMeshImport>(jsonString);
    }

    /// <summary>
    ///     Optimized the data from Unity based on a real world use case scenario.
    /// </summary>
    /// <returns>Optimized navigation mesh with added information for pathing calculations</returns>
    private static NavMeshOptimized OptimizeNavMesh(NavMeshImport import)
    {
        #region Check Vertices and Indices for overlap

        List<Vector3> verts = import.GetVertices();
        List<int> indices = import.GetIndices();
        Dictionary<Vector2, List<int>> vertsByPos = new Dictionary<Vector2, List<int>>();

        const float groupSize = 5f;

        for (int i = 0; i < verts.Count; i++)
        {
            Vector3 v = verts[i];
            Vector2 id = new Vector2((float)Math.Floor(v.X / groupSize),
                (float)Math.Floor(v.Z / groupSize));

            if (vertsByPos.TryGetValue(id, out List<int> outList))
                outList.Add(i);
            else
                vertsByPos.Add(id, new List<int> { i });
        }

        CheckOverlap(verts, indices, vertsByPos, groupSize);

        #endregion

        #region Create first iteration of NavTriangles

        List<NavMeshTriangle> triangles = new List<NavMeshTriangle>();
        Dictionary<int, List<int>> trianglesByVertexId = new Dictionary<int, List<int>>();

        SetupNavTriangles(verts, indices, triangles, trianglesByVertexId);

        triangles = SetupNeighbors(triangles, trianglesByVertexId);

        #endregion

        #region Check neighbor connections

        Vector3 cleanPoint = import.GetCleanPoint();
        int closestVert = 0;
        float closestDistance = cleanPoint.QuickSquareDistance(verts[closestVert]);

        for (int i = 1; i < verts.Count; i++)
        {
            float d = cleanPoint.QuickSquareDistance(verts[i]);
            if (d >= closestDistance)
                continue;

            if (trianglesByVertexId.TryGetValue(i, out List<int> value) &&
                !value.Any(t => triangles[t].Neighbors.Count > 0))
                continue;

            closestDistance = d;
            closestVert = i;
        }

        List<int> connected = new List<int>(), toCheck = new List<int>();
        toCheck.AddRange(trianglesByVertexId[closestVert]);

        while (toCheck.Count > 0)
        {
            int index = toCheck[0];
            NavMeshTriangle navTriangle = triangles[index];
            toCheck.RemoveAt(0);
            connected.Add(index);
            foreach (int n in navTriangle.Neighbors)
                if (!toCheck.Contains(n) && !connected.Contains(n))
                    toCheck.Add(n);
        }

        #endregion

        #region Fill holes and final iteration of NavTriangles

        List<Vector3> fixedVertices = new List<Vector3>();
        List<int> fixedIndices = new List<int>();
        Dictionary<int, List<int>> fixedTrianglesByVertexId = new Dictionary<int, List<int>>();

        foreach (NavMeshTriangle t in connected.Select(i => triangles[i]))
        {
            int aId = t.Vertices[0], bId = t.Vertices[1], cId = t.Vertices[2];
            Vector3 a = verts[aId], b = verts[bId], c = verts[cId];

            if (!fixedVertices.Contains(a))
                fixedVertices.Add(a);

            if (!fixedVertices.Contains(b))
                fixedVertices.Add(b);

            if (!fixedVertices.Contains(c))
                fixedVertices.Add(c);

            fixedIndices.Add(fixedVertices.IndexOf(a));
            fixedIndices.Add(fixedVertices.IndexOf(b));
            fixedIndices.Add(fixedVertices.IndexOf(c));

            Vector2 id = new Vector2((int)Math.Floor(a.X / groupSize),
                (int)Math.Floor(a.Z / groupSize));
            if (vertsByPos.TryGetValue(id, out List<int> outListA))
            {
                if (outListA.Contains(fixedVertices.IndexOf(a)))
                    outListA.Add(fixedVertices.IndexOf(a));
            }
            else
            {
                vertsByPos.Add(id, new List<int> { fixedVertices.IndexOf(a) });
            }

            id = new Vector2((int)Math.Floor(b.X / groupSize), (int)Math.Floor(b.Z / groupSize));
            if (vertsByPos.TryGetValue(id, out List<int> outListB))
            {
                if (outListB.Contains(fixedVertices.IndexOf(b)))
                    outListB.Add(fixedVertices.IndexOf(b));
            }
            else
            {
                vertsByPos.Add(id, new List<int> { fixedVertices.IndexOf(b) });
            }

            id = new Vector2((int)Math.Floor(c.X / groupSize), (int)Math.Floor(c.Z / groupSize));
            if (vertsByPos.TryGetValue(id, out List<int> outListC))
            {
                if (outListC.Contains(fixedVertices.IndexOf(c)))
                    outListC.Add(fixedVertices.IndexOf(c));
            }
            else
            {
                vertsByPos.Add(id, new List<int> { fixedVertices.IndexOf(c) });
            }
        }

        FillHoles(fixedVertices, fixedIndices);

        List<NavMeshTriangle> fixedTriangles = new List<NavMeshTriangle>();

        SetupNavTriangles(fixedVertices, fixedIndices, fixedTriangles, fixedTrianglesByVertexId);

        fixedTriangles = SetupNeighbors(fixedTriangles, fixedTrianglesByVertexId);

        for (int i = 0; i < fixedTriangles.Count; i++)
            fixedTriangles[i].SetBorderWidth(fixedVertices, fixedTriangles);

        #endregion

        return new NavMeshOptimized();
    }

    /// <summary>
    ///     Creates new NavTriangles from the indices
    /// </summary>
    /// <param name="verts">3D vertices</param>
    /// <param name="indices">Each pair of threes indicate one triangle</param>
    /// <param name="triangles">List for the triangles to be added to</param>
    /// <param name="trianglesByVertexId">Each triangle will be assigned to each relevant vertex for optimization later</param>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static void SetupNavTriangles(IReadOnlyList<Vector3> verts, IReadOnlyList<int> indices,
        ICollection<NavMeshTriangle> triangles, IDictionary<int, List<int>> trianglesByVertexId)
    {
        for (int i = 0; i < indices.Count; i += 3)
        {
            int a = indices[i], b = indices[i + 1], c = indices[i + 2];
            NavMeshTriangle triangle = new NavMeshTriangle(i / 3, a, b, c, verts[a], verts[b], verts[c]);

            triangles.Add(triangle);

            int tId = triangles.Count - 1;

            if (trianglesByVertexId.TryGetValue(a, out List<int> list))
            {
                if (!list.Contains(tId))
                    list.Add(tId);
            }
            else
            {
                trianglesByVertexId.Add(a, new List<int>() { tId });
            }

            if (trianglesByVertexId.TryGetValue(b, out list))
            {
                if (!list.Contains(tId))
                    list.Add(tId);
            }
            else
            {
                trianglesByVertexId.Add(b, new List<int>() { tId });
            }

            if (trianglesByVertexId.TryGetValue(c, out list))
            {
                if (!list.Contains(tId))
                    list.Add(tId);
            }
            else
            {
                trianglesByVertexId.Add(c, new List<int>() { tId });
            }
        }
    }

    /// <summary>
    ///     Setup the connected neighbors for each NavTriangle
    /// </summary>
    /// <param name="triangles">The triangles to check</param>
    /// <param name="trianglesByVertexId">A list of triangle ids based on a vertex id</param>
    /// <returns>The current triangle list now with neighbors set</returns>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static List<NavMeshTriangle> SetupNeighbors(List<NavMeshTriangle> triangles,
        IReadOnlyDictionary<int, List<int>> trianglesByVertexId)
    {
        for (int i = 0; i < triangles.Count; i++)
        {
            List<int> neighbors = new List<int>();
            List<int> possibleNeighbors = new List<int>();
            possibleNeighbors.AddRange(trianglesByVertexId[triangles[i].Vertices[0]]);
            possibleNeighbors.AddRange(trianglesByVertexId[triangles[i].Vertices[1]]);
            possibleNeighbors.AddRange(trianglesByVertexId[triangles[i].Vertices[2]]);

            possibleNeighbors = possibleNeighbors.Where(t => t != i).ToList();

            foreach (int t in possibleNeighbors)
            {
                if (triangles[i].Vertices.SharedBetween(triangles[t].Vertices).Length == 2)
                    neighbors.Add(t);

                if (triangles.Count == 3)
                    break;
            }

            triangles[i].SetNeighborIDs(neighbors.ToArray());
        }

        return triangles;
    }

    /// <summary>
    ///     Check each vertex and if its close to another then remove the other and replace any indices containing it with the
    ///     current one.
    ///     If the other vertex is in a triangle with the current then remove said triangle.
    /// </summary>
    /// <param name="verts">3D vertices</param>
    /// <param name="indices">Each pair of threes indicate one triangle</param>
    /// <param name="vertsByPos">List of vertex ids based on the divided floor int of x and z value</param>
    /// <param name="groupSize">Division value for creating the vertex groupings</param>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static void CheckOverlap(List<Vector3> verts, List<int> indices, Dictionary<Vector2, List<int>> vertsByPos,
        float groupSize)
    {
        const float overlapCheckDistance = .3f;
        const float divided = 20f;
        Dictionary<int, List<int>> removed = new Dictionary<int, List<int>>();
        for (int original = 0; original < verts.Count; original++)
        {
            if (removed.TryGetValue((int)Math.Floor(original / divided), out List<int> removedList))
                if (removedList.Contains(original))
                    continue;

            Vector2 id = new Vector2((int)Math.Floor(verts[original].X / groupSize),
                (int)Math.Floor(verts[original].Z / groupSize));
            List<int> toCheck = new List<int>();
            if (vertsByPos.TryGetValue(id, out List<int> l1))
                toCheck.AddRange(l1);
            if (vertsByPos.TryGetValue(id + new Vector2(-1, -1), out List<int> l2))
                toCheck.AddRange(l2);
            if (vertsByPos.TryGetValue(id + new Vector2(-1, 0), out List<int> l3))
                toCheck.AddRange(l3);
            if (vertsByPos.TryGetValue(id + new Vector2(-1, 1), out List<int> l4))
                toCheck.AddRange(l4);
            if (vertsByPos.TryGetValue(id + new Vector2(0, -1), out List<int> l5))
                toCheck.AddRange(l5);
            if (vertsByPos.TryGetValue(id + new Vector2(0, 1), out List<int> l6))
                toCheck.AddRange(l6);
            if (vertsByPos.TryGetValue(id + new Vector2(1, -1), out List<int> l7))
                toCheck.AddRange(l7);
            if (vertsByPos.TryGetValue(id + new Vector2(1, 0), out List<int> l8))
                toCheck.AddRange(l8);
            if (vertsByPos.TryGetValue(id + new Vector2(1, 1), out List<int> l9))
                toCheck.AddRange(l9);

            toCheck = toCheck.Where(x => x != original).ToList();

            foreach (int other in toCheck)
            {
                if (removed.TryGetValue((int)Math.Floor(other / divided), out removedList))
                    if (removedList.Contains(other))
                        continue;

                if (Vector3.Distance(verts[original], verts[other]) > overlapCheckDistance)
                    continue;

                if (removed.TryGetValue((int)Math.Floor(other / divided), out removedList))
                    removedList.Add(other);
                else
                    removed.Add((int)Math.Floor(other / divided), new List<int> { other });

                for (int indicesIndex = 0; indicesIndex < indices.Count; indicesIndex++)
                    if (indices[indicesIndex] == other)
                        indices[indicesIndex] = original;
            }
        }

        List<int> toRemove = removed.Values.SelectMany(l => l).OrderBy(x => -x).ToList();
        foreach (int index in toRemove)
        {
            vertsByPos[
                new Vector2((int)Math.Floor(verts[index].X / groupSize),
                    (int)Math.Floor(verts[index].Z / groupSize))].Remove(index);
            verts.RemoveAt(index);

            for (int j = 0; j < indices.Count; j++)
                if (indices[j] >= index)
                    indices[j] = indices[j] - 1;
        }

        for (int i = indices.Count - 1; i >= 0; i--)
        {
            if (i % 3 != 0)
                continue;

            if (indices[i] != indices[i + 1] && indices[i] != indices[i + 2] && indices[i + 1] != indices[i + 2] &&
                indices[i] < verts.Count && indices[i + 1] < verts.Count && indices[i + 2] < verts.Count)
                continue;

            indices.RemoveAt(i);
            indices.RemoveAt(i);
            indices.RemoveAt(i);
        }
    }

    /// <summary>
    ///     Fill any holes that might have appeared by checking overlap.
    ///     If any three vertexes are directly connected to each other without having a matching triangle then add one.
    /// </summary>
    /// <param name="verts">3D vertices</param>
    /// <param name="indices">Each pair of threes indicate one triangle</param>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static void FillHoles(IList<Vector3> verts, IList<int> indices)
    {
        Dictionary<int, List<int>> connectionsByIndex = new Dictionary<int, List<int>>();
        Dictionary<int, List<int>> indicesByIndex = new Dictionary<int, List<int>>();
        for (int i = 0; i < verts.Count; i++)
        {
            connectionsByIndex.Add(i, new List<int>());
            indicesByIndex.Add(i, new List<int>());
        }

        for (int i = 0; i < indices.Count; i += 3)
        {
            if (!connectionsByIndex[indices[i]].Contains(indices[i + 1]))
                connectionsByIndex[indices[i]].Add(indices[i + 1]);
            if (!connectionsByIndex[indices[i]].Contains(indices[i + 2]))
                connectionsByIndex[indices[i]].Add(indices[i + 2]);

            if (!connectionsByIndex[indices[i + 1]].Contains(indices[i]))
                connectionsByIndex[indices[i + 1]].Add(indices[i]);
            if (!connectionsByIndex[indices[i + 1]].Contains(indices[i + 2]))
                connectionsByIndex[indices[i + 1]].Add(indices[i + 2]);

            if (!connectionsByIndex[indices[i + 2]].Contains(indices[i]))
                connectionsByIndex[indices[i + 2]].Add(indices[i]);
            if (!connectionsByIndex[indices[i + 2]].Contains(indices[i + 1]))
                connectionsByIndex[indices[i + 2]].Add(indices[i + 1]);

            int[] arr = { indices[i], indices[i + 1], indices[i + 2] };
            indicesByIndex[indices[i]].AddRange(arr);
            indicesByIndex[indices[i + 1]].AddRange(arr);
            indicesByIndex[indices[i + 2]].AddRange(arr);
        }

        for (int i = 0; i < verts.Count; i++)
        {
            Vector2 p = verts[i].Xz();

            for (int j = 0; j < indices.Count; j += 3)
            {
                if (indices[j] == i || indices[j + 1] == i || indices[j + 2] == i)
                    continue;

                Vector2 a = verts[indices[j]].Xz(), b = verts[indices[j + 1]].Xz(), c = verts[indices[j + 2]].Xz();

                if (!MathC.PointWithinTriangle2D(p, a, b, c))
                    continue;

                Vector2 close1 = MathC.ClosetPointOnLine(p, a, b),
                    close2 = MathC.ClosetPointOnLine(p, a, b),
                    close3 = MathC.ClosetPointOnLine(p, a, b);

                Vector2 close;
                if (Vector2.Distance(close1, p) < Vector2.Distance(close2, p) &&
                    Vector2.Distance(close1, p) < Vector2.Distance(close3, p))
                    close = close1;
                else if (Vector2.Distance(close2, p) < Vector2.Distance(close3, p))
                    close = close2;
                else
                    close = close3;

                Vector2 offset = close - p;

                verts[i] += (offset.Normalize() * (offset.Length() + .01f)).Xyz();
            }
        }

        for (int original = 0; original < verts.Count; original++)
        {
            List<int> originalConnections = connectionsByIndex[original];

            for (int otherIndex = 0; otherIndex < originalConnections.Count; otherIndex++)
            {
                int other = originalConnections[otherIndex];

                if (other <= original)
                    continue;

                for (int finalIndex = otherIndex + 1; finalIndex < originalConnections.Count; finalIndex++)
                {
                    int final = originalConnections[finalIndex];

                    if (final <= original || !connectionsByIndex[final].Contains(other))
                        continue;

                    bool denied = false;

                    Vector2 a = verts[original].Xz(), b = verts[other].Xz(), c = verts[final].Xz();
                    Vector2 center = Vector2.Lerp(Vector2.Lerp(a, b, .5f), c, .5f);

                    float minX = Math.Min(Math.Min(a.X, b.X), c.X),
                        minY = Math.Min(Math.Min(a.Y, b.Y), c.Y),
                        maxX = Math.Max(Math.Max(a.X, b.X), c.X),
                        maxY = Math.Max(Math.Max(a.Y, b.Y), c.Y);

                    for (int x = 0; x < indices.Count; x += 3)
                    {
                        List<int> checkArr = new List<int> { indices[x], indices[x + 1], indices[x + 2] };
                        if (checkArr.Contains(original) && checkArr.Contains(other) && checkArr.Contains(final))
                        {
                            //The triangle already exists
                            denied = true;
                            break;
                        }

                        Vector2 aP = verts[checkArr[0]].Xz(),
                            bP = verts[checkArr[1]].Xz(),
                            cP = verts[checkArr[2]].Xz();

                        //Bounding
                        if (maxX < Math.Min(Math.Min(aP.X, bP.X), cP.X) ||
                            maxY < Math.Min(Math.Min(aP.Y, bP.Y), cP.Y) ||
                            minX > Math.Max(Math.Max(aP.X, bP.X), cP.X) ||
                            minY > Math.Max(Math.Max(aP.Y, bP.Y), cP.Y))
                            continue;

                        //One of the new triangle points is within an already existing triangle
                        if (MathC.PointWithinTriangle2D(center, aP, bP, cP) ||
                            MathC.PointWithinTriangle2D(a, aP, bP, cP) ||
                            MathC.PointWithinTriangle2D(b, aP, bP, cP) ||
                            MathC.PointWithinTriangle2D(c, aP, bP, cP))
                        {
                            denied = true;
                            break;
                        }

                        if (MathC.TriangleIntersect2D(a, b, c, aP, bP, cP))
                        {
                            denied = true;
                            break;
                        }
                    }

                    if (denied)
                        continue;

                    indices.Add(original);
                    indices.Add(other);
                    indices.Add(final);
                }
            }
        }
    }
}

internal readonly struct NavMeshImport
{
    private readonly Vector3 cleanPoint;
    private readonly List<Vector3> vertices;
    private readonly List<int> indices;

    public NavMeshImport(Vector3 cleanPoint, List<Vector3> vertices, List<int> indices)
    {
        this.cleanPoint = cleanPoint;
        this.vertices = vertices;
        this.indices = indices;
    }

    public List<Vector3> GetVertices() => this.vertices;

    public List<int> GetIndices() => this.indices;

    public Vector3 GetCleanPoint() => this.cleanPoint;
}

internal struct NavMeshOptimized
{
    #region Values

    private Vector2[] vertices2D;
    private float[] verticesY;

    private NavMeshTriangle[] triangles;

    private Dictionary<Vector2, List<int>> trianglesByVertexPosition;

    /// <summary>
    ///     Index of vertex returns all NavTriangles containing the vertex id.
    /// </summary>
    private List<List<int>> triangleByVertexId;

    private const float GroupDivision = 10f;

    #endregion

    #region Getters

    public Vector3[] GetVertices()
    {
        Vector3[] result = new Vector3[this.verticesY.Length];
        for (int i = 0; i < this.verticesY.Length; i++)
            result[i] = new Vector3(this.vertices2D[i].X, this.verticesY[i], this.vertices2D[i].Y);
        return result;
    }

    public NavMeshTriangle[] GetTriangles() => this.triangles;

    #endregion

    #region Setters

    public void SetValues(Vector3[] vertices, NavMeshTriangle[] navTriangles)
    {
        this.triangles = new NavMeshTriangle[navTriangles.Length];
        for (int i = 0; i < navTriangles.Length; i++)
            this.triangles[i] = navTriangles[i];

        this.vertices2D = new Vector2[vertices.Length];
        this.verticesY = new float[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            float x = vertices[i].X, y = vertices[i].Y, z = vertices[i].Z;
            this.vertices2D[i] = new Vector2(x, z);
            this.verticesY[i] = y;
        }

        this.trianglesByVertexPosition = new Dictionary<Vector2, List<int>>();

        this.triangleByVertexId = new List<List<int>>();
        for (int i = 0; i < vertices.Length; i++)
            this.triangleByVertexId.Add(new List<int>());

        foreach (NavMeshTriangle t in navTriangles)
        {
            int a = t.Vertices[0], b = t.Vertices[1], c = t.Vertices[2];
            Vector2 aV = new Vector2((int)Math.Floor(this.vertices2D[a].X / GroupDivision),
                    (int)Math.Floor(this.vertices2D[a].Y / GroupDivision)),
                bV = new Vector2((int)Math.Floor(this.vertices2D[b].X / GroupDivision),
                    (int)Math.Floor(this.vertices2D[b].Y / GroupDivision)),
                cV = new Vector2((int)Math.Floor(this.vertices2D[c].X / GroupDivision),
                    (int)Math.Floor(this.vertices2D[c].Y / GroupDivision));

            if (this.trianglesByVertexPosition.TryGetValue(aV, out List<int> list))
                list.Add(t.Id);
            else
                this.trianglesByVertexPosition.Add(aV, new List<int> { t.Id });

            if (this.trianglesByVertexPosition.TryGetValue(bV, out list))
                list.Add(t.Id);
            else
                this.trianglesByVertexPosition.Add(bV, new List<int> { t.Id });

            if (this.trianglesByVertexPosition.TryGetValue(cV, out list))
                list.Add(t.Id);
            else
                this.trianglesByVertexPosition.Add(cV, new List<int> { t.Id });

            this.triangleByVertexId[a].Add(t.Id);
            this.triangleByVertexId[b].Add(t.Id);
            this.triangleByVertexId[c].Add(t.Id);
        }
    }

    #endregion
}

internal readonly struct NavMeshTriangle
{
    private readonly int a, b, c;

    private readonly Vector3 ab, bc, ca;

    private readonly List<int> neighborIDs;
    private readonly List<float> widthDistanceBetweenNeighbor;

    public NavMeshTriangle(int id, int a, int b, int c, params Vector3[] verts)
    {
        this.Id = id;
        this.a = a;
        this.b = b;
        this.c = c;

        this.ab = (verts[0] - verts[1]).Normalize();
        this.bc = (verts[1] - verts[2]).Normalize();
        this.ca = (verts[2] - verts[0]).Normalize();

        this.neighborIDs = new List<int>();
        this.widthDistanceBetweenNeighbor = new List<float>();
    }

    #region Getters

    public int Id { get; }

    public int[] Vertices => new int[] { this.a, this.b, this.c };

    public List<int> Neighbors => this.neighborIDs;

    #endregion

    #region Setters

    public void SetNeighborIDs(int[] set)
    {
        this.neighborIDs.Clear();
        foreach (int t in set)
            if (!this.neighborIDs.Contains(t))
                this.neighborIDs.Add(t);
    }

    public void SetBorderWidth(List<Vector3> verts, List<NavMeshTriangle> triangles)
    {
        if (this.neighborIDs.Count == 0)
            return;

        for (int i = 0; i < this.neighborIDs.Count; i++)
            this.widthDistanceBetweenNeighbor.Add(0f);

        for (int i = 0; i < this.neighborIDs.Count; i++)
        {
            int otherId = this.neighborIDs[i];
            NavMeshTriangle other = triangles[otherId];
            int[] ids = other.Vertices.SharedBetween(this.Vertices);

            if (ids.Length != 2)
                continue;

            float dist = Vector3.Distance(verts[ids[0]], verts[ids[1]]);

            if (i + 2 < this.neighborIDs.Count)
            {
                int connectedBorderNeighbor = -1;
                if (other.neighborIDs.Contains(this.neighborIDs[i + 2]))
                    connectedBorderNeighbor = i + 2;
                else if (other.neighborIDs.Contains(this.neighborIDs[i + 1]))
                    connectedBorderNeighbor = i + 1;

                if (connectedBorderNeighbor > -1)
                {
                    ids = triangles[this.neighborIDs[connectedBorderNeighbor]].Vertices
                        .SharedBetween(this.Vertices);
                    if (ids.Length == 2)
                    {
                        dist += Vector3.Distance(verts[ids[0]], verts[ids[1]]);
                        this.widthDistanceBetweenNeighbor[connectedBorderNeighbor] = dist;
                    }
                }
            }
            else if (i + 1 < this.neighborIDs.Count)
            {
                if (other.neighborIDs.Contains(this.neighborIDs[i + 1]))
                {
                    ids = triangles[this.neighborIDs[i + 1]].Vertices.SharedBetween(this.Vertices);
                    if (ids.Length == 2)
                    {
                        dist += Vector3.Distance(verts[ids[0]], verts[ids[1]]);
                        this.widthDistanceBetweenNeighbor[i + 1] = dist;
                    }
                }
            }

            this.widthDistanceBetweenNeighbor[i] = dist;
        }
    }

    #endregion
}

public static class Extensions
{
    public static Vector2 Normalize(this Vector2 v) =>
        v / v.Length();

    public static Vector3 Normalize(this Vector3 v) =>
        v / v.Length();

    public static Vector2 Xz(this Vector3 v) =>
        new Vector2(v.X, v.Y);

    public static Vector3 Xyz(this Vector2 v, float y = 0)
        => new Vector3(v.X, y, v.Y);
}

public static class MathC
{
    public static bool PointWithinTriangle2D(Vector2 point, Vector2 a, Vector2 b, Vector2 c,
        float tolerance = .001f)
    {
        float w1 = (a.X * (c.Y - a.Y) + (point.Y - a.Y) * (c.X - a.X) - point.X * (c.Y - a.Y)) /
                   ((b.Y - a.Y) * (c.X - a.X) - (b.X - a.X) * (c.Y - a.Y));

        float w2 = (point.Y - a.Y - w1 * (b.Y - a.Y)) /
                   (c.Y - a.Y);

        return w1 >= tolerance && w2 >= tolerance && w1 + w2 <= 1f - tolerance;
    }

    public static bool TriangleIntersect2D(Vector2 a1, Vector2 a2, Vector2 a3, Vector2 b1, Vector2 b2, Vector2 b3)
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

    private static bool LineIntersect2D(Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2)
    {
        //Line1
        float a1 = end1.Y - start1.Y;
        float b1 = start1.X - end1.X;
        float c1 = a1 * start1.X + b1 * start1.Y;

        //Line2
        float a2 = end2.Y - start2.Y;
        float b2 = start2.X - end2.X;
        float c2 = a2 * start2.X + b2 * start2.Y;

        float denominator = a1 * b2 - a2 * b1;

        if (denominator == 0)
            return false;

        Vector2 point = new Vector2((b2 * c1 - b1 * c2) / denominator, (a1 * c2 - a2 * c1) / denominator);

        if (point == start1 || point == end1 ||
            point == start2 || point == end2)
            return false;

        const float tolerance = .001f;

        return point.X > MathF.Min(start1.X, end1.X) + tolerance &&
               point.X < MathF.Max(start1.X, end1.X) - tolerance &&
               point.X > MathF.Min(start2.X, end2.X) + tolerance &&
               point.X < MathF.Max(start2.X, end2.X) - tolerance &&
               point.Y > MathF.Min(start1.Y, end1.Y) + tolerance &&
               point.Y < MathF.Max(start1.Y, end1.Y) - tolerance &&
               point.Y > MathF.Min(start2.Y, end2.Y) + tolerance &&
               point.Y < MathF.Max(start2.Y, end2.Y) - tolerance;
    }

    public static Vector2 ClosetPointOnLine(Vector2 point, Vector2 start, Vector2 end)
    {
        //Get heading
        Vector2 heading = end - start;
        float magnitudeMax = heading.Length();
        heading = heading.Normalize();

        //Do projection from the point but clamp it
        Vector2 lhs = point - start;
        float dotP = Vector2.Dot(lhs, heading);
        dotP = Math.Clamp(dotP, 0f, magnitudeMax);

        return start + heading * dotP;
    }

    public static T[] SharedBetween<T>(this T[] target, T[] other, int max = -1)
    {
        List<T> result = new List<T>();

        foreach (T a in target)
        {
            if (a == null)
                continue;

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

    public static float QuickSquareDistance(this Vector3 point1, Vector3 point2) => (point1 - point2).LengthSquared();
}