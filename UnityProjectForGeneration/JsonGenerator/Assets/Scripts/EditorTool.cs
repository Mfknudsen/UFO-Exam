// ReSharper disable AccessToStaticMemberViaDerivedType
// ReSharper disable PossibleNullReferenceException

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Cysharp.Threading.Tasks;
using Unity.AI.Navigation;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.SceneManagement;

public abstract class EditorTool
{
    private const float OVERLAP_CHECK_DISTANCE = .3f;

    [MenuItem("Tools/Run JSON Generator")]
    private static void JsonGenerator()
    {
        for (int i = 0; i < EditorSceneManager.loadedRootSceneCount; i++)
            EditorSceneManager.CloseScene(EditorSceneManager.GetSceneAt(i), false);

        string[] sceneNameLetter = { "S", "M", "L" }, folderName = { "Small", "Medium", "Large" };

        //Going from the Unity projects /Assets file path to /JsonFiles in the GitHub project for easier access for the C#, C++ and Python optimization scripts.
        string path = Application.dataPath;
        path = Directory.GetParent(path).FullName;
        path = Directory.GetParent(path).FullName;
        path = Directory.GetParent(path).FullName;
        path = $"{path}/JsonFiles/";
        Debug.Log(path);

        for (int sizeIndex = 0; sizeIndex < 3; sizeIndex++)
        for (int numberIndex = 1; numberIndex <= 5; numberIndex++)
        {
            string sceneName = $"{sceneNameLetter[sizeIndex]} {numberIndex}";
            EditorSceneManager.OpenScene($"Assets/Scenes/{folderName[sizeIndex]}/{sceneName}.unity");

            NavMeshTriangulation navMeshTriangulation =
                BuildNavMeshTriangulation(GameObject.FindObjectOfType<NavMeshSurface>());
            GameObject cleanPoint = GameObject.Find("Clean Point");

            NavigationMesh navigationMesh = BakeNavmesh();

            NavMeshExport export = new NavMeshExport(cleanPoint.transform.position,
                navMeshTriangulation.vertices,
                navMeshTriangulation.indices,
                navigationMesh.verticesY.Length,
                navigationMesh.triangles.Length,
                navigationMesh.indices.Length);

            Debug.Log(sceneName);
            Debug.Log(cleanPoint.transform.position);
            Debug.Log($"Vertex count: {navMeshTriangulation.vertices.Length}");
            Debug.Log($"Index count:  {navMeshTriangulation.indices.Length}");

            Debug.Log($"F v: {navigationMesh.verticesY.Length}");
            Debug.Log($"F t: {navigationMesh.triangles.Length}");
            Debug.Log($"F i: {navigationMesh.indices.Length}");

            if (!Directory.Exists($"{path}{sceneName}.txt"))
                File.Create($"{path}{sceneName}.txt").Dispose();

            string json = JsonUtility.ToJson(export);

            File.WriteAllText($"{path}{sceneName}.txt", json);

            EditorSceneManager.CloseScene(EditorSceneManager.GetActiveScene(), false);
        }
    }

    private static NavMeshTriangulation BuildNavMeshTriangulation(
        NavMeshSurface surface)
    {
        surface.BuildNavMesh();
        NavMeshTriangulation navmesh = NavMesh.CalculateTriangulation();
        surface.RemoveData();
        surface.navMeshData = null;
        return navmesh;
    }

    private class NavMeshExport
    {
        public Vector3 cleanPoint;
        public List<float> x, y, z;
        public List<int> indices;
        public int finalVertexCount, finalTriangleCount, finalIndicesCount;

        public NavMeshExport(Vector3 cleanPoint, Vector3[] verts, int[] indices, int finalVertexCount,
            int finalTriangleCount, int finalIndicesCount)
        {
            this.cleanPoint = cleanPoint;
            this.x = new List<float>();
            this.y = new List<float>();
            this.z = new List<float>();
            foreach (Vector3 v in verts)
            {
                this.x.Add(v.x);
                this.y.Add(v.y);
                this.z.Add(v.z);
            }

            this.indices = new List<int>();
            foreach (int index in indices)
                this.indices.Add(index);

            this.finalVertexCount = finalVertexCount;
            this.finalTriangleCount = finalTriangleCount;
            this.finalIndicesCount = finalIndicesCount;
        }
    }

    /// <summary>
    ///     Bake a custom Navmesh for use with the custom Navmesh agents.
    /// </summary>
    private static NavigationMesh BakeNavmesh()
    {
        NavigationMesh result = null;
        GameObject tileSubController = GameObject.Find("Navigation Area");
        if (BakedEditorManager.IsBakeRunning || tileSubController == null)
            return result;

        //Stops other processes from starting at the same time.
        BakedEditorManager.SetRunning(true);

        //Title for the progressbar displayed during the baking process.
        string editorProgressParTitle = "Baking Navigation: " + tileSubController.scene.name;

        //Find and disable all editor only objects.
        GameObject[] editorGameObjects =
            GameObject.FindGameObjectsWithTag("EditorOnly").Where(o => o.activeSelf).ToArray();
        foreach (GameObject editorGameObject in editorGameObjects)
            editorGameObject.SetActive(false);
        try
        {
            #region Load neighbor scenes and build navmesh triangulation

            //Construct Navigation Mesh and setup custom navmesh logic
            EditorUtility.DisplayProgressBar(editorProgressParTitle, "Calculation Navigation Mesh Triangulation",
                0f);

            //Use Unitys build in method for creating a navigation mesh and store the information. 
            NavMeshTriangulation navmesh =
                BuildNavMeshTriangulation(tileSubController.GetComponent<NavMeshSurface>());

            #endregion

            #region Check Vertices and Indices for overlap

            //First iteration of the values to be stored.
            List<Vector3> verts = navmesh.vertices.ToList();
            List<int> indices = navmesh.indices.ToList();
            List<int> areas = navmesh.areas.ToList();
            Dictionary<Vector2Int, List<int>> vertsByPos = new Dictionary<Vector2Int, List<int>>();

            //Group vertices by their 2D (x,z) positions for faster checking later on.
            const float groupSize = 5f;
            for (int i = 0; i < verts.Count; i++)
            {
                Vector3 v = verts[i];
                Vector2Int id = new Vector2Int(Mathf.FloorToInt(v.x / groupSize),
                    Mathf.FloorToInt(v.z / groupSize));

                if (vertsByPos.TryGetValue(id, out List<int> outList))
                    outList.Add(i);
                else
                    vertsByPos.Add(id, new List<int> { i });
            }

            //Check if some vertices is so close to another that one can be removed while the other takes over the removed ones connection.
            CheckOverlap(verts, indices, vertsByPos, groupSize, editorProgressParTitle);

            #endregion

            #region Create first iteration of NavTriangles

            EditorUtility.DisplayProgressBar(editorProgressParTitle, "Creating first iteration NavTriangles", 0f);
            List<NavTriangle> triangles = new List<NavTriangle>();
            Dictionary<int, List<int>> trianglesByVertexID = new Dictionary<int, List<int>>();

            SetupNavTriangles(indices, areas, triangles, trianglesByVertexID, editorProgressParTitle);

            triangles = SetupNeighbors(triangles, trianglesByVertexID, editorProgressParTitle, "First");

            #endregion

            #region Check neighbor connections

            EditorUtility.DisplayProgressBar(editorProgressParTitle, "Checking NavTriangle neighbor connections",
                .5f);
            Vector3 cleanPoint = GameObject.Find("Clean Point").transform.position;
            int closestVert = 0;
            float closestDistance = cleanPoint.QuickSquareDistance(verts[closestVert]);

            for (int i = 1; i < verts.Count; i++)
            {
                float d = cleanPoint.QuickSquareDistance(verts[i]);
                if (d >= closestDistance)
                    continue;

                if (trianglesByVertexID.TryGetValue(i, out List<int> value) &&
                    !value.Any(t => triangles[t].Neighbors.Count > 0))
                    continue;

                closestDistance = d;
                closestVert = i;
            }

            List<int> connected = new List<int>(), toCheck = new List<int>();
            toCheck.AddRange(trianglesByVertexID[closestVert]);

            while (toCheck.Count > 0)
            {
                int index = toCheck[0];
                NavTriangle navTriangle = triangles[index];
                toCheck.RemoveAt(0);
                connected.Add(index);
                foreach (int n in navTriangle.Neighbors)
                    if (!toCheck.Contains(n) && !connected.Contains(n))
                        toCheck.Add(n);

                EditorUtility.DisplayProgressBar(editorProgressParTitle,
                    "Checking NavTriangle neighbor connections", .5f + .5f / triangles.Count * connected.Count);
            }

            #endregion

            #region Fill holes and final iteration of NavTriangles

            List<Vector3> fixedVertices = new List<Vector3>();
            List<int> fixedIndices = new List<int>(), fixedAreas = new List<int>();
            Dictionary<int, List<int>> fixedTrianglesByVertexID = new Dictionary<int, List<int>>();

            foreach (NavTriangle t in connected.Select(i => triangles[i]))
            {
                int aID = t.Vertices[0], bID = t.Vertices[1], cID = t.Vertices[2];
                Vector3 a = verts[aID], b = verts[bID], c = verts[cID];

                if (!fixedVertices.Contains(a))
                    fixedVertices.Add(a);

                if (!fixedVertices.Contains(b))
                    fixedVertices.Add(b);

                if (!fixedVertices.Contains(c))
                    fixedVertices.Add(c);

                fixedIndices.Add(fixedVertices.IndexOf(a));
                fixedIndices.Add(fixedVertices.IndexOf(b));
                fixedIndices.Add(fixedVertices.IndexOf(c));

                fixedAreas.Add(t.Area);

                Vector2Int id = new Vector2Int(Mathf.FloorToInt(a.x / groupSize),
                    Mathf.FloorToInt(a.z / groupSize));
                if (vertsByPos.TryGetValue(id, out List<int> outListA))
                {
                    if (outListA.Contains(fixedVertices.IndexOf(a)))
                        outListA.Add(fixedVertices.IndexOf(a));
                }
                else
                {
                    vertsByPos.Add(id, new List<int> { fixedVertices.IndexOf(a) });
                }

                id = new Vector2Int(Mathf.FloorToInt(b.x / groupSize), Mathf.FloorToInt(b.z / groupSize));
                if (vertsByPos.TryGetValue(id, out List<int> outListB))
                {
                    if (outListB.Contains(fixedVertices.IndexOf(b)))
                        outListB.Add(fixedVertices.IndexOf(b));
                }
                else
                {
                    vertsByPos.Add(id, new List<int> { fixedVertices.IndexOf(b) });
                }

                id = new Vector2Int(Mathf.FloorToInt(c.x / groupSize), Mathf.FloorToInt(c.z / groupSize));
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

            FillHoles(fixedVertices, fixedAreas, fixedIndices, editorProgressParTitle);

            List<NavTriangle> fixedTriangles = new List<NavTriangle>();

            SetupNavTriangles(fixedIndices, fixedAreas, fixedTriangles,
                fixedTrianglesByVertexID, editorProgressParTitle);

            fixedTriangles = SetupNeighbors(fixedTriangles, fixedTrianglesByVertexID, editorProgressParTitle,
                "Final");

            for (int i = 0; i < fixedTriangles.Count; i++)
            {
                fixedTriangles[i].SetBorderWidth(fixedVertices, fixedTriangles);
                EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle, "Setting border width",
                    1f / fixedTriangles.Count * (i + 1));
            }

            #endregion

            result = new NavigationMesh(fixedVertices.ToArray(), fixedIndices.ToArray(), fixedTriangles.ToArray());
        }
        catch (Exception e)
        {
            //Displayed progressbar contains a cancel button which when used should throw an exception with the message: "Cancel".
            if (e.Message.Equals("Cancel"))
            {
                Debug.Log("Baking navmesh was canceled");
            }
            else
            {
                //If an exception is thrown without the message: "Cancel" then it is an error.
                Debug.LogError("Baking Navigation Mesh Failed");
                Debug.LogError(e);
            }
        }


        //The progressbar is no longer needed.
        EditorUtility.ClearProgressBar();

        //Save the the current scene
        EditorSceneManager.SaveScene(tileSubController.gameObject.scene);

        //Other process may start now.
        BakedEditorManager.SetRunning(false);

        return result;
    }

    /// <summary>
    ///     Creates new NavTriangles from the indices
    /// </summary>
    /// <param name="verts">3D vertices</param>
    /// <param name="indices">Each pair of threes indicate one triangle</param>
    /// <param name="areas">Area values for each triangle</param>
    /// <param name="triangles">List for the triangles to be added to</param>
    /// <param name="trianglesByVertexID">Each triangle will be assigned to each relevant vertex for optimization later</param>
    /// <param name="editorProgressParTitle">Editor loading bar title</param>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static void SetupNavTriangles(IReadOnlyList<int> indices,
        IReadOnlyList<int> areas,
        ICollection<NavTriangle> triangles, IDictionary<int, List<int>> trianglesByVertexID,
        string editorProgressParTitle)
    {
        for (int i = 0; i < indices.Count; i += 3)
        {
            int a = indices[i], b = indices[i + 1], c = indices[i + 2];
            NavTriangle triangle = new NavTriangle(a, b, c, areas[i / 3]);

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

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Creating NavTriangles: {i / 3f} / {indices.Count / 3f}",
                    1f / (indices.Count / 3f) * (i / 3f)))
                throw new Exception("Cancel");
        }
    }

    /// <summary>
    ///     Setup the connected neighbors for each NavTriangle
    /// </summary>
    /// <param name="triangles">The triangles to check</param>
    /// <param name="trianglesByVertexID">A list of triangle ids based on a vertex id</param>
    /// <param name="editorProgressParTitle">Editor loading bar title</param>
    /// <param name="iteration">This will run multiple times during baking. Will help the user know which step</param>
    /// <returns>The current triangle list now with neighbors set</returns>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static List<NavTriangle> SetupNeighbors(List<NavTriangle> triangles,
        IReadOnlyDictionary<int, List<int>> trianglesByVertexID, string editorProgressParTitle, string iteration)
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

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Setting up NavTriangles neighbors. {iteration} iteration: {i} / {triangles.Count}",
                    1f / triangles.Count * i))
                throw new Exception("Cancel");
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
    /// <param name="vertsByPos">List of vertex ids grouped based on the divided floor int of x and z value</param>
    /// <param name="groupSize">Division value for creating the vertex groupings</param>
    /// <param name="editorProgressParTitle">Editor progressbar title</param>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static void CheckOverlap(List<Vector3> verts, List<int> indices,
        Dictionary<Vector2Int, List<int>> vertsByPos,
        float groupSize, string editorProgressParTitle)
    {
        //The ids of vertices to be removed, grouped index.
        Dictionary<int, List<int>> removed = new Dictionary<int, List<int>>();
        for (int currentVertIndex = 0; currentVertIndex < verts.Count; currentVertIndex++)
        {
            //Check if the current vertex id is part of the to be removed.
            if (removed.TryGetValue(Mathf.FloorToInt(currentVertIndex / groupSize), out List<int> removedList))
                //If its to be removed then dont check this vertex.
                if (removedList.Contains(currentVertIndex))
                    continue;

            //2D id of the vertex based on its x and z values and grouped by group size.
            Vector2Int id = new Vector2Int(Mathf.FloorToInt(verts[currentVertIndex].x / groupSize),
                Mathf.FloorToInt(verts[currentVertIndex].z / groupSize));

            //Get the 
            List<int> toCheck = new List<int>();
            for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
                if (vertsByPos.TryGetValue(id + new Vector2Int(x, y), out List<int> list))
                    toCheck.AddRange(list);

            toCheck = toCheck.Where(x => x != currentVertIndex).ToList();

            foreach (int other in toCheck)
            {
                if (removed.TryGetValue(Mathf.FloorToInt(other / groupSize), out removedList))
                    if (removedList.Contains(other))
                        continue;

                if (Vector3.Distance(verts[currentVertIndex], verts[other]) > OVERLAP_CHECK_DISTANCE)
                    continue;

                if (removed.TryGetValue(Mathf.FloorToInt(other / groupSize), out removedList))
                    removedList.Add(other);
                else
                    removed.Add(Mathf.FloorToInt(other / groupSize), new List<int> { other });

                for (int indicesIndex = 0; indicesIndex < indices.Count; indicesIndex++)
                    if (indices[indicesIndex] == other)
                        indices[indicesIndex] = currentVertIndex;
            }

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Checking vertex overlap: {currentVertIndex} / {verts.Count}",
                    1f / verts.Count * currentVertIndex))
                throw new Exception("Cancel");
        }

        List<int> toRemove = removed.Values.SelectMany(l => l).OrderBy(x => -x).ToList();
        for (int i = 0; i < toRemove.Count; i++)
        {
            int index = toRemove[i];
            vertsByPos[
                new Vector2Int(Mathf.FloorToInt(verts[index].x / groupSize),
                    Mathf.FloorToInt(verts[index].z / groupSize))].Remove(index);
            verts.RemoveAt(index);

            for (int j = 0; j < indices.Count; j++)
                if (indices[j] >= index)
                    indices[j] = indices[j] - 1;

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Removing overlapping vertices: {i} / {toRemove.Count}", 1f / toRemove.Count * i))
                throw new Exception("Cancel");
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

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Correcting indices indexes: {indices.Count - i} / {verts.Count}",
                    1f / verts.Count * (indices.Count - i)))
                throw new Exception("Cancel");
        }
    }

    /// <summary>
    ///     Fill any holes that might have appeared by checking overlap.
    ///     If any three vertexes are directly connected to each other without having a matching triangle then add one.
    /// </summary>
    /// <param name="verts">3D vertices</param>
    /// <param name="areas">When a new triangle is created then add an area value as well</param>
    /// <param name="indices">Each pair of threes indicate one triangle</param>
    /// <param name="editorProgressParTitle">Editor loading bar title</param>
    /// <exception cref="Exception">Throws "Cancel" if the user cancels the progress</exception>
    private static void FillHoles(IList<Vector3> verts, ICollection<int> areas, IList<int> indices,
        string editorProgressParTitle)
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

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Collecting vertex connections and indices: {i} / {indices.Count}", 1f / indices.Count * i))
                throw new Exception("Cancel");
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

                verts[i] += (offset.normalized * (offset.magnitude + .01f)).ToV3(0);
            }

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Checking point overlap with triangles: {i} / {verts.Count}", 1f / verts.Count * i))
                throw new Exception("Cancel");
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

                    Vector2 a = verts[original].XZ(), b = verts[other].XZ(), c = verts[final].XZ();
                    Vector2 center = Vector2.Lerp(Vector2.Lerp(a, b, .5f), c, .5f);

                    float minX = Mathf.Min(Mathf.Min(a.x, b.x), c.x),
                        minY = Mathf.Min(Mathf.Min(a.y, b.y), c.y),
                        maxX = Mathf.Max(Mathf.Max(a.x, b.x), c.x),
                        maxY = Mathf.Max(Mathf.Max(a.y, b.y), c.y);

                    for (int x = 0; x < indices.Count; x += 3)
                    {
                        List<int> checkArr = new List<int> { indices[x], indices[x + 1], indices[x + 2] };
                        if (checkArr.Contains(original) && checkArr.Contains(other) && checkArr.Contains(final))
                        {
                            //The triangle already exists
                            denied = true;
                            break;
                        }

                        Vector2 aP = verts[checkArr[0]].XZ(),
                            bP = verts[checkArr[1]].XZ(),
                            cP = verts[checkArr[2]].XZ();

                        //Bounding
                        if (maxX < Mathf.Min(Mathf.Min(aP.x, bP.x), cP.x) ||
                            maxY < Mathf.Min(Mathf.Min(aP.y, bP.y), cP.y) ||
                            minX > Mathf.Max(Mathf.Max(aP.x, bP.x), cP.x) ||
                            minY > Mathf.Max(Mathf.Max(aP.y, bP.y), cP.y))
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

                    areas.Add(0);
                    indices.Add(original);
                    indices.Add(other);
                    indices.Add(final);
                }
            }

            if (EditorUtility.DisplayCancelableProgressBar(editorProgressParTitle,
                    $"Finding and filling holes: {original} / {verts.Count}", 1f / verts.Count * original))
                throw new Exception("Cancel");
        }
    }


    public static class BakedEditorManager
    {
        #region Values

        private static bool isBakeRunning;

        #endregion

        #region Getters

        public static bool IsBakeRunning =>
            isBakeRunning;

        #endregion

        #region Setters

        public static void SetRunning(bool set) =>
            isBakeRunning = set;

        #endregion
    }

    public struct NavTriangle
    {
        #region Values

        private int A, B, C;

        private int area;

        private List<int> neighborIDs;
        private List<float> widthDistanceBetweenNeighbor;

        #endregion

        #region Build In States

        public NavTriangle(int A, int B, int C, int area)
        {
            this.A = A;
            this.B = B;
            this.C = C;

            this.area = area;

            this.neighborIDs = new List<int>();
            this.widthDistanceBetweenNeighbor = new List<float>();
        }

        #endregion

        #region Getters

        public readonly int[] Vertices => new int[] { this.A, this.B, this.C };

        public readonly List<int> Neighbors => this.neighborIDs;

        public readonly int Area => this.area;

        #endregion

        #region Setters

#if UNITY_EDITOR
        public void SetNeighborIDs(int[] set)
        {
            this.neighborIDs.Clear();
            for (int i = 0; i < set.Length; i++)
                if (!this.neighborIDs.Contains(set[i]))
                    this.neighborIDs.Add(set[i]);
        }

#endif

        #endregion

        #region In

#if UNITY_EDITOR

        public void SetBorderWidth(List<Vector3> verts, List<NavTriangle> triangles)
        {
            if (this.neighborIDs.Count == 0)
                return;

            for (int i = 0; i < this.neighborIDs.Count; i++)
                this.widthDistanceBetweenNeighbor.Add(0f);

            for (int i = 0; i < this.neighborIDs.Count; i++)
            {
                int otherID = this.neighborIDs[i];
                NavTriangle other = triangles[otherID];
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

#endif

        #endregion
    }

    public sealed class NavigationMesh
    {
        #region Values

        public Vector2[] vertices2D;
        public float[] verticesY;

        public int[] indices;

        public NavTriangle[] triangles;

        #endregion

        public NavigationMesh(Vector3[] vertices, int[] indices, NavTriangle[] navTriangles)
        {
            this.indices = indices;
            this.triangles = new NavTriangle[navTriangles.Length];
            for (int i = 0; i < navTriangles.Length; i++)
                this.triangles[i] = navTriangles[i];

            this.vertices2D = new Vector2[vertices.Length];
            this.verticesY = new float[vertices.Length];
            for (int i = 0; i < vertices.Length; i++)
            {
                this.vertices2D[i] = new Vector2(vertices[i].x, vertices[i].z);
                this.verticesY[i] = vertices[i].y;
            }
        }
    }
}