// See https://aka.ms/new-console-template for more information

#pragma warning disable CS8600 // Converting null literal or possible null value to non-nullable type.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.IO;
using System.Linq;
using System.Numerics;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

namespace CSharpOptimizer
{
#pragma warning disable CS8604 // Possible null reference argument.
#pragma warning disable CS8602 // Dereference of a possibly null reference.

// ReSharper disable CollectionNeverQueried.Local
// ReSharper disable NotAccessedField.Local
// ReSharper disable RedundantExplicitArrayCreation

    internal static class ProgramMain
    {
        private const float OVERLAP_CHECK_DISTANCE = .3f;

        public static void Main()
        {
            const int averageCount = 1;

            string[] fileLetter = new[] { "S", "M", "L" };
            DirectoryInfo di = new DirectoryInfo(Directory.GetCurrentDirectory());
            string folderPath =
                $@"{di.Parent.Parent.Parent.Parent}\JsonFiles\";

            Console.Write($"Using json text files from folder:\n{folderPath}\n");

            for (int letterIndex = 0; letterIndex < 3; letterIndex++)
            for (int numberIndex = 1; numberIndex <= 5; numberIndex++)
            {
                Console.WriteLine($"Optimization for: {fileLetter[letterIndex]} {numberIndex}");
                NavMeshImport navMeshImport =
                    LoadJsonToNavMeshImport($"{folderPath}{fileLetter[letterIndex]} {numberIndex}");

                float totalTime = 0;
                for (int i = 0; i < averageCount; i++)
                {
                    List<Vector3> vertices = new List<Vector3>();
                    foreach (Vector3 v in navMeshImport.GetVertices())
                        vertices.Add(v);

                    List<int> indices = new List<int>();
                    foreach (int index in navMeshImport.GetIndices())
                        indices.Add(index);

                    Console.WriteLine($"Check {i + 1}");
                    Console.WriteLine($"Start vertex count: {navMeshImport.GetVertices().Count}");
                    Console.WriteLine($"Start indices count: {navMeshImport.GetIndices().Count}\n");
                    Stopwatch stopwatch = Stopwatch.StartNew();
                    NavMeshOptimized navMeshOptimized =
                        OptimizeNavMesh(navMeshImport.GetCleanPoint(), vertices, indices);
                    float m = stopwatch.ElapsedMilliseconds;
                    totalTime += m;

                    Console.WriteLine($"Final vertex count: {navMeshOptimized.GetVertices().Length}");
                    Console.WriteLine($"Final indices count: {navMeshOptimized.GetIndices().Length}");
                    Console.WriteLine($"Final triangle count: {navMeshOptimized.GetTriangles().Length}\n");

                    Console.WriteLine(
                        $"Vertex count match: {navMeshOptimized.GetVertices().Length == navMeshImport.FVertex()} | {navMeshImport.FVertex() - navMeshOptimized.GetVertices().Length}");
                    Console.WriteLine(
                        $"Indices count match: {navMeshOptimized.GetIndices().Length == navMeshImport.FIndices()} | {navMeshImport.FIndices() - navMeshOptimized.GetIndices().Length}");
                    Console.WriteLine(
                        $"Triangle count match: {navMeshOptimized.GetTriangles().Length == navMeshImport.FTriangle()} | {navMeshImport.FTriangle() - navMeshOptimized.GetTriangles().Length}\n");

                    Console.WriteLine($"FVertex: {navMeshImport.FVertex()}");
                    Console.WriteLine($"FIndices: {navMeshImport.FIndices()}");
                    Console.WriteLine($"FTriangle: {navMeshImport.FTriangle()}");

                    Console.WriteLine($"Time: {m}(ms)");
                    Console.WriteLine($"Time: {m / 1000}(s)\n");
                }

                if (averageCount == 1)
                    continue;

                Console.WriteLine($"Repeat count: {averageCount}");

                Console.WriteLine($"Total time for repeats: {totalTime}(ms)");
                Console.WriteLine($"Total time for repeats: {totalTime / 1000}(s)");

                Console.WriteLine(
                    $"Average time for {fileLetter[letterIndex]} {numberIndex}: {totalTime / averageCount}(ms)");
                Console.WriteLine(
                    $"Average time for {fileLetter[letterIndex]} {numberIndex}: {totalTime / averageCount / 1000}(s)\n");
            }
        }

        /// <summary>
        ///     Load data generated in Unity and stored into a Json file.
        /// </summary>
        /// <returns>Import struct containing relevant information of the navigation mesh</returns>
        private static NavMeshImport LoadJsonToNavMeshImport(string fileName)
        {
            using StreamReader r = new StreamReader($"{fileName}.txt");

            string jsonString = r.ReadToEnd();
            JObject data = JsonConvert.DeserializeObject<JObject>(jsonString);

            List<Vector3> vertices = new List<Vector3>();
            for (int i = 0; i < data["x"].Count(); i++)
                vertices.Add(new Vector3(data["x"][i].Value<float>(), data["y"][i].Value<float>(),
                    data["z"][i].Value<float>()));

            Vector3 cleanPoint = new Vector3(data["cleanPoint"]["x"].Value<float>(),
                data["cleanPoint"]["y"].Value<float>(),
                data["cleanPoint"]["z"].Value<float>());

            List<int> indices = new List<int>();
            for (int i = 0; i < data["indices"].Count(); i++)
                indices.Add(data["indices"][i].Value<int>());

            return new NavMeshImport(cleanPoint, vertices, indices,
                data["finalVertexCount"].Value<int>(),
                data["finalTriangleCount"].Value<int>(),
                data["finalIndicesCount"].Value<int>());
        }

        /// <summary>
        ///     Optimized the data from Unity based on a real world use case scenario.
        /// </summary>
        /// <returns>Optimized navigation mesh with added information for pathing calculations</returns>
        private static NavMeshOptimized OptimizeNavMesh(Vector3 cleanPoint, List<Vector3> verts, List<int> indices)
        {
            #region Check Vertices and Indices for overlap

            Dictionary<Vector2Int, List<int>> vertsByPosition = new Dictionary<Vector2Int, List<int>>();

            const float groupSize = 5f;

            for (int i = 0; i < verts.Count; i++)
            {
                Vector3 v = verts[i];
                Vector2Int id = new Vector2Int((int)MathF.Floor(v.X / groupSize),
                    (int)MathF.Floor(v.Z / groupSize));

                if (vertsByPosition.TryGetValue(id, out List<int> outList))
                    outList.Add(i);
                else
                    vertsByPosition.Add(id, new List<int> { i });
            }

            CheckOverlap(verts, indices, vertsByPosition, groupSize);

            #endregion

            #region Create first iteration of NavTriangles

            List<NavMeshTriangle> triangles = new List<NavMeshTriangle>();
            Dictionary<int, List<int>> trianglesByVertexId = new Dictionary<int, List<int>>();
            for (int i = 0; i < verts.Count; i++)
                trianglesByVertexId.Add(i, new List<int>());

            SetupNavTriangles(indices, triangles, trianglesByVertexId);

            triangles = SetupNeighbors(triangles, trianglesByVertexId);

            #endregion

            #region Check neighbor connections

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

            foreach (NavMeshTriangle t in connected.Select(i => triangles[i]))
            foreach (int tVertex in t.Vertices)
            {
                if (!fixedVertices.Contains(verts[tVertex]))
                    fixedVertices.Add(verts[tVertex]);

                fixedIndices.Add(fixedVertices.IndexOf(verts[tVertex]));

                Vector2Int id = new Vector2Int((int)Math.Floor(verts[tVertex].X / groupSize),
                    (int)Math.Floor(verts[tVertex].Z / groupSize));
                if (vertsByPosition.TryGetValue(id, out List<int> outListA))
                {
                    if (outListA.Contains(fixedVertices.IndexOf(verts[tVertex])))
                        outListA.Add(fixedVertices.IndexOf(verts[tVertex]));
                }
                else
                {
                    vertsByPosition.Add(id, new List<int> { fixedVertices.IndexOf(verts[tVertex]) });
                }
            }

            FillHoles(fixedVertices, fixedIndices);

            List<NavMeshTriangle> fixedTriangles = new List<NavMeshTriangle>();
            Dictionary<int, List<int>> fixedTrianglesByVertexId = new Dictionary<int, List<int>>();
            for (int i = 0; i < fixedVertices.Count; i++)
                fixedTrianglesByVertexId.Add(i, new List<int>());

            SetupNavTriangles(fixedIndices, fixedTriangles, fixedTrianglesByVertexId);

            fixedTriangles = SetupNeighbors(fixedTriangles, fixedTrianglesByVertexId);

            for (int i = 0; i < fixedTriangles.Count; i++)
                fixedTriangles[i].SetBorderWidth(fixedVertices, fixedTriangles);

            #endregion

            NavMeshOptimized result = new NavMeshOptimized();
            result.SetValues(fixedVertices.ToArray(), fixedIndices.ToArray(), fixedTriangles.ToArray());
            return result;
        }

        /// <summary>
        ///     Creates new NavTriangles from the indices
        /// </summary>
        /// <param name="verts">3D vertices</param>
        /// <param name="indices">Each pair of threes indicate one triangle</param>
        /// <param name="triangles">List for the triangles to be added to</param>
        /// <param name="trianglesByVertexId">Each triangle will be assigned to each relevant vertex for optimization later</param>
        private static void SetupNavTriangles(IReadOnlyList<int> indices,
            ICollection<NavMeshTriangle> triangles, IDictionary<int, List<int>> trianglesByVertexID)
        {
            for (int i = 0; i < indices.Count; i += 3)
            {
                int a = indices[i], b = indices[i + 1], c = indices[i + 2];
                NavMeshTriangle triangle = new NavMeshTriangle(i / 3, a, b, c);

                triangles.Add(triangle);

                int tID = triangles.Count - 1;

                if (trianglesByVertexID.TryGetValue(a, out List<int> list))
                {
                    if (!list.Contains(tID))
                        list.Add(tID);
                }
                else
                {
                    trianglesByVertexID.Add(a, new List<int>() { tID });
                }

                if (trianglesByVertexID.TryGetValue(b, out list))
                {
                    if (!list.Contains(tID))
                        list.Add(tID);
                }
                else
                {
                    trianglesByVertexID.Add(b, new List<int>() { tID });
                }

                if (trianglesByVertexID.TryGetValue(c, out list))
                {
                    if (!list.Contains(tID))
                        list.Add(tID);
                }
                else
                {
                    trianglesByVertexID.Add(c, new List<int>() { tID });
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
            IReadOnlyDictionary<int, List<int>> trianglesByVertexID)
        {
            for (int i = 0; i < triangles.Count; i++)
            {
                List<int> neighbors = new List<int>();
                List<int> possibleNeighbors = new List<int>();
                possibleNeighbors.AddRange(trianglesByVertexID[triangles[i].Vertices[0]]);
                possibleNeighbors.AddRange(trianglesByVertexID[triangles[i].Vertices[1]]);
                possibleNeighbors.AddRange(trianglesByVertexID[triangles[i].Vertices[2]]);

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
        private static void CheckOverlap(List<Vector3> verts, List<int> indices,
            Dictionary<Vector2Int, List<int>> vertsByPos,
            float groupSize)
        {
            //The ids of vertices to be removed, grouped index.
            Dictionary<int, List<int>> removed = new Dictionary<int, List<int>>();
            for (int currentVertIndex = 0; currentVertIndex < verts.Count; currentVertIndex++)
            {
                //Check if the current vertex id is part of the to be removed.
                if (removed.TryGetValue((int)MathF.Floor(currentVertIndex / groupSize), out List<int> removedList))
                    //If its to be removed then dont check this vertex.
                    if (removedList.Contains(currentVertIndex))
                        continue;

                //2D id of the vertex based on its x and z values and grouped by group size.
                Vector2Int id = new Vector2Int((int)MathF.Floor(verts[currentVertIndex].X / groupSize),
                    (int)MathF.Floor(verts[currentVertIndex].Z / groupSize));

                //Get the 
                List<int> toCheck = new List<int>();
                for (int x = -1; x <= 1; x++)
                for (int y = -1; y <= 1; y++)
                    if (vertsByPos.TryGetValue(id + new Vector2Int(x, y), out List<int> list))
                        toCheck.AddRange(list);

                toCheck = toCheck.Where(x => x != currentVertIndex).ToList();

                foreach (int other in toCheck)
                {
                    if (removed.TryGetValue((int)MathF.Floor(other / groupSize), out removedList))
                        if (removedList.Contains(other))
                            continue;

                    if (Vector3.Distance(verts[currentVertIndex], verts[other]) > OVERLAP_CHECK_DISTANCE)
                        continue;

                    if (removed.TryGetValue((int)MathF.Floor(other / groupSize), out removedList))
                        removedList.Add(other);
                    else
                        removed.Add((int)MathF.Floor(other / groupSize), new List<int> { other });

                    for (int indicesIndex = 0; indicesIndex < indices.Count; indicesIndex++)
                        if (indices[indicesIndex] == other)
                            indices[indicesIndex] = currentVertIndex;
                }
            }

            List<int> toRemove = removed.Values.SelectMany(l => l).OrderBy(x => -x).ToList();
            for (int i = 0; i < toRemove.Count; i++)
            {
                int index = toRemove[i];
                vertsByPos[
                    new Vector2Int((int)MathF.Floor(verts[index].X / groupSize),
                        (int)MathF.Floor(verts[index].Z / groupSize))].Remove(index);
                verts.RemoveAt(index);

                for (int j = 0; j < indices.Count; j++)
                    if (indices[j] >= index)
                        indices[j] = indices[j] - 1;
            }

            for (int i = indices.Count - 1; i >= 0; i--)
            {
                if (i % 3 != 0)
                    continue;

                if (indices[i] == indices[i + 1] || indices[i] == indices[i + 2] || indices[i + 1] == indices[i + 2] ||
                    indices[i] >= verts.Count || indices[i + 1] >= verts.Count || indices[i + 2] >= verts.Count)
                {
                    indices.RemoveAt(i);
                    indices.RemoveAt(i);
                    indices.RemoveAt(i);
                }
            }
        }

        /// <summary>
        ///     Fill any holes that might have appeared by checking overlap.
        ///     If any three vertexes are directly connected to each other without having a matching triangle then add one.
        /// </summary>
        /// <param name="verts">3D vertices</param>
        /// <param name="indices">Each pair of threes indicate one triangle</param>
        /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
        [SuppressMessage("ReSharper.DPA", "DPA0002: Excessive memory allocations in SOH",
            MessageId = "type: System.Int32[]; size: 102MB")]
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
                Vector2 p = verts[i].XZ();

                for (int j = 0; j < indices.Count; j += 3)
                {
                    if (indices[j] == i || indices[j + 1] == i || indices[j + 2] == i)
                        continue;

                    Vector2 a = verts[indices[j]].XZ(), b = verts[indices[j + 1]].XZ(), c = verts[indices[j + 2]].XZ();

                    if (!MathC.PointWithinTriangle2DWithTolerance(p, a, b, c))
                        continue;

                    Vector2 close1 = MathC.ClosetPointOnLine(p, a, b),
                        close2 = MathC.ClosetPointOnLine(p, a, c),
                        close3 = MathC.ClosetPointOnLine(p, b, b);

                    Vector2 close;
                    if (Vector2.Distance(close1, p) < Vector2.Distance(close2, p) &&
                        Vector2.Distance(close1, p) < Vector2.Distance(close3, p))
                        close = close1;
                    else if (Vector2.Distance(close2, p) < Vector2.Distance(close3, p))
                        close = close2;
                    else
                        close = close3;

                    Vector2 offset = close - p;

                    verts[i] += (offset.Normalize() * (MathC.Magnitude(offset) + .01f)).ToV3(0);
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

                        Vector2 a = XZ(verts[original]), b = XZ(verts[other]), c = XZ(verts[final]);
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

                            Vector2 aP = XZ(verts[checkArr[0]]),
                                bP = XZ(verts[checkArr[1]]),
                                cP = XZ(verts[checkArr[2]]);

                            //Bounding
                            if (maxX < Math.Min(Math.Min(aP.X, bP.X), cP.X) ||
                                maxY < Math.Min(Math.Min(aP.Y, bP.Y), cP.Y) ||
                                minX > Math.Max(Math.Max(aP.X, bP.X), cP.X) ||
                                minY > Math.Max(Math.Max(aP.Y, bP.Y), cP.Y))
                                continue;

                            //One of the new triangle points is within an already existing triangle
                            if (MathC.PointWithinTriangle2DWithTolerance(center, aP, bP, cP) ||
                                MathC.PointWithinTriangle2DWithTolerance(a, aP, bP, cP) ||
                                MathC.PointWithinTriangle2DWithTolerance(b, aP, bP, cP) ||
                                MathC.PointWithinTriangle2DWithTolerance(c, aP, bP, cP))
                            {
                                denied = true;
                                break;
                            }

                            if (!MathC.TriangleIntersect2D(a, b, c, aP, bP, cP))
                                continue;

                            denied = true;
                            break;
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

        private static Vector2 XZ(Vector3 vert) =>
            new Vector2(vert.X, vert.Z);
    }

    internal class NavMeshExport
    {
#pragma warning disable CS0649 // Field is never assigned to, and will always have its default value
        // ReSharper disable once InconsistentNaming
        public Vector3 cleanPoint;

#pragma warning disable CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider declaring as nullable.
        // ReSharper disable once InconsistentNaming
        public List<float> x,
            // ReSharper disable once InconsistentNaming
            y,
            // ReSharper disable once InconsistentNaming
            z;

        // ReSharper disable once InconsistentNaming
        public List<int> indices;
#pragma warning restore CS8618 // Non-nullable field must contain a non-null value when exiting constructor. Consider declaring as nullable.
#pragma warning restore CS0649 // Field is never assigned to, and will always have its default value
    }

    internal readonly struct NavMeshImport
    {
        /// <summary>
        ///     Part of the clean up of data is removing any triangles which cannot be reached by any NPC.
        ///     The clean point is use to find the closest triangle which we want to be part of the usable move area.
        /// </summary>
        private readonly Vector3 cleanPoint;

        private readonly List<int> indices;

        private readonly List<Vector3> vertices;

        private readonly int finalVertexCount, finalTriangleCount, finalIndicesCount;

        public NavMeshImport(Vector3 cleanPoint, List<Vector3> vertices, List<int> indices,
            int finalVertexCount, int finalTriangleCount, int finalIndicesCount)
        {
            this.cleanPoint = cleanPoint;
            this.indices = indices;
            this.vertices = vertices;

            this.finalVertexCount = finalVertexCount;
            this.finalTriangleCount = finalTriangleCount;
            this.finalIndicesCount = finalIndicesCount;
        }

        public List<Vector3> GetVertices() => this.vertices;

        public List<int> GetIndices() => this.indices;

        public Vector3 GetCleanPoint() => this.cleanPoint;

        public int FVertex() => this.finalVertexCount;

        public int FTriangle() => this.finalTriangleCount;

        public int FIndices() => this.finalIndicesCount;
    }

    internal struct NavMeshOptimized
    {
        #region Values

        private Vector2[] vertices2D;
        private float[] verticesY;

        private int[] indices;

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

        public int[] GetIndices() => this.indices;

        #endregion

        #region Setters

        public void SetValues(Vector3[] vertices, int[] indices, NavMeshTriangle[] navTriangles)
        {
            this.indices = indices;
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
        private readonly int id;
        private readonly int a, b, c;

        private readonly List<int> neighborIDs;
        private readonly List<float> widthDistanceBetweenNeighbor;

        public NavMeshTriangle(int id, int a, int b, int c)
        {
            this.id = id;
            this.a = a;
            this.b = b;
            this.c = c;

            this.neighborIDs = new List<int>();
            this.widthDistanceBetweenNeighbor = new List<float>();
        }

        #region Getters

        // ReSharper disable once ConvertToAutoProperty
        public int Id => this.id;

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
        public static float QuickSquareDistance(this Vector3 point1, Vector3 point2) =>
            (point1 - point2).SqrMagnitude();

        public static float SqrMagnitude(this Vector3 v) => v.X * v.X + v.Y * v.Y + v.Z * v.Z;

        public static Vector2 XZ(this Vector3 target) => new Vector2(target.X, target.Z);

        public static Vector3 ToV3(this Vector2 t, float y) => new Vector3(t.X, y, t.Y);

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

        public static Vector2 Normalize(this Vector2 v) => v / MathC.Magnitude(v);
    }

    internal struct Vector2Int
    {
        public int x, y;

        public Vector2Int(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        public static Vector2Int operator +(Vector2Int a, Vector2Int b)
            => new Vector2Int(a.x + b.x, a.y + b.y);

        public static Vector2Int operator -(Vector2Int a, Vector2Int b)
            => new Vector2Int(a.x - b.x, a.y - b.y);

        public static bool operator ==(Vector2Int a, Vector2Int b) => a.x == b.x && a.y == b.y;

        public static bool operator !=(Vector2Int a, Vector2Int b) => a.x != b.x || a.y != b.y;

        public bool Equals(Vector2Int other) => this.x == other.x && this.y == other.y;

        public override bool Equals(object obj) => obj is Vector2Int other && Equals(other);

        public override int GetHashCode() => HashCode.Combine(this.x, this.y);
    }

    public static class MathC
    {
        public static bool LineIntersect2DWithTolerance(Vector2 start1, Vector2 end1, Vector2 start2, Vector2 end2)
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

        public static bool PointWithinTriangle2DWithTolerance(Vector2 point, Vector2 a, Vector2 b, Vector2 c,
            float tolerance = .001f)
        {
            float s1 = c.Y - a.Y + 0.0001f;
            float s2 = c.X - a.X;
            float s3 = b.Y - a.Y;
            float s4 = point.Y - a.Y;

            float w1 = (a.X * s1 + s4 * s2 - point.X * s1) / (s3 * s2 - (b.X - a.X + 0.0001f) * s1);
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
            float magnitudeMax = Magnitude(heading);
            heading.Normalize();

            //Do projection from the point but clamp it
            Vector2 lhs = point - start;
            float dotP = Vector2.Dot(lhs, heading);
            dotP = Math.Clamp(dotP, 0f, magnitudeMax);

            return start + heading * dotP;
        }

        public static float Magnitude(Vector2 v) => MathF.Sqrt(v.X * v.X + v.Y * v.Y);
        public static float Magnitude(Vector3 v) => MathF.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);

        public static float FastSqrt(float number)
        {
            if (number < 2)
                return number;

            //Repeat for better approximation
            float a = 1000;
            float b = number / a;
            a = (a + b) * .5f;

            b = number / a;
            a = (a + b) * .5f;

            b = number / a;
            a = (a + b) * .5f;

            b = number / a;
            a = (a + b) * .5f;

            b = number / a;
            a = (a + b) * .5f;

            return a;
        }

        public static float FastSqrtDistance(Vector2 v) => FastSqrt(v.X * v.X + v.Y * v.Y);

        public static bool IsPointLeftToVector(Vector2 lineA, Vector2 lineB, Vector2 point)
            => (lineB.X - lineA.X) * (point.Y - lineA.Y) -
                (lineB.Y - lineA.Y) * (point.X - lineA.X) > 0;

        public static float QuickCircleIntersectCircleArea(Vector3 center1, Vector3 center2, float radius1,
            float radius2, float height1, float height2)
        {
            if (center1.Y > center2.Y + height2 || center2.Y > center1.Y + height1)
                return 0;

            float squaredRadius1 = radius1 * radius1,
                squaredRadius2 = radius2 * radius2;

            float c = FastSqrt((center2.X - center1.X) * (center2.X - center1.X) +
                               (center2.Z - center1.Z) * (center2.Z - center1.Z));

            float phi = MathF.Acos((squaredRadius1 + c * c - squaredRadius2) / (2 * radius1 * c)) * 2;
            float theta = MathF.Acos((squaredRadius2 + c * c - squaredRadius1) / (2 * radius2 * c)) * 2;

            float area1 = 0.5f * theta * squaredRadius2 - 0.5f * squaredRadius2 * MathF.Sin(theta);
            float area2 = 0.5f * phi * squaredRadius1 - 0.5f * squaredRadius1 * MathF.Sin(phi);

            return (area1 + area2) * MathF.Abs(height1 - height2);
        }
    }
}