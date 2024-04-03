#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include <map>

using json = nlohmann::json;

using namespace std;
using namespace chrono;

namespace fs = filesystem;

#include "NavMeshImport.h"
#include "NavMeshOptimized.h"
#include "MathC.h"
#include "Vector2Int.h"
#include "Vector3.h"


void CheckOverlap(vector<Vector2> *verts, vector<int> *indices,
                  map<Vector2Int, vector<int>> *vertsByPosition, float size);

void SetupNavTriangles(vector<Vector2> *verts, vector<int> *indices, vector<NavMeshTriangle> *triangles,
                       map<int, vector<int>> *trianglesByVertexId);

void SetupNeighbors(vector<NavMeshTriangle> *triangles, map<int, vector<int>> *trianglesByVertexId);

void FillHoles(vector<Vector2> *verts, vector<int> *indices);

bool contains(vector<int> *v, int *target);

bool contains(vector<Vector2> *v, Vector2 *target);

vector<float> &offsetVector(Vector2 *v, int x, int y);

void insertRange(vector<int> *target, vector<int> *from);

int indexOf(vector<Vector2> *list, Vector2 *element);

vector<float> &XZ(Vector2 *vector);

vector<float> &XYZ(Vector2 *v);

NavMeshImport *loadJsonToNavMeshImport(fs::path &file) {
    cout << "   Importing navigation mesh from file:" << "\n";
    cout << "   " << file << "\n";

    ifstream str(file);
    json js = json::parse(str);

    vector<float> &cleanPoint = *new vector<float>{js["cleanPoint"]["x"],
                                                   js["cleanPoint"]["y"],
                                                   js["cleanPoint"]["z"]};
    cout << "   Clean Point {" << cleanPoint[0] << " , " << cleanPoint[1] << " , " << cleanPoint[2] << "}" << "\n";

    vector<vector<float>> &vertexPoints = *new vector<vector<float>>;

    for (int i = 0; i < js["x"].size(); ++i) {
        vertexPoints.push_back(vector<float>{js["x"][i], js["y"][i], js["z"][i]});
    }

    cout << "   Vertex count: " << vertexPoints.size() << "\n";

    vector<int> &indices = *new vector<int>;

    for (const auto &item: js["indices"])
        indices.push_back(item);

    cout << "   Indices count: " << indices.size() << "\n\n";

    return new NavMeshImport(cleanPoint, vertexPoints, indices);
}

NavMeshOptimized &OptimizeNavMesh(Vector3 cleanPoint, vector<Vector3> verts, vector<int> indices) {
#pragma region Check Vertices and Indices for overlap

    map<Vector2Int, vector<int>> vertsByPosition = *new map<Vector2Int, vector<int>>();

    const float groupSize = 5.0f;

    for (int i = 0; i < verts.size(); i++) {
        Vector3 v = verts[i];
        Vector2Int id = *new Vector2Int((int) MathC::Floor(v.x / groupSize),
                                        (int) MathC::Floor(v.z / groupSize));

        if (vertsByPosition.TryGetValue(id, out vector<int> outList))
        outList.Add(i);
        else
        vertsByPosition.Add(id, new vector<int>{i});
    }

    CheckOverlap(verts, indices, vertsByPosition, groupSize);

#pragma endregion

#pragma region Create first iteration of NavTriangles

    vector<NavMeshTriangle> triangles = new vector<NavMeshTriangle>();
    map<int, vector<int>> trianglesByVertexId = new map<int, vector<int>>();
    for (int i = 0; i < verts.size(); i++)
        trianglesByVertexId.Add(i, new vector<int>());

    SetupNavTriangles(indices, triangles, trianglesByVertexId);

    triangles = SetupNeighbors(triangles, trianglesByVertexId);

#pragma endregion

#pragma region Check neighbor connections

    int closestVert = 0;
    float closestDistance = cleanPoint.QuickSquareDistance(verts[closestVert]);

    for (int i = 1; i < verts.size(); i++) {
        float d = cleanPoint.QuickSquareDistance(verts[i]);
        if (d >= closestDistance)
            continue;

        if (trianglesByVertexId.TryGetValue(i, out vector<int> value) &&
        !value.Any(t = > triangles[t].Neighbors.size > 0))
        continue;

        closestDistance = d;
        closestVert = i;
    }

    vector<int> connected = new vector<int>(), toCheck = new vector<int>();
    toCheck.AddRange(trianglesByVertexId[closestVert]);

    while (toCheck.size > 0) {
        int index = toCheck[0];
        NavMeshTriangle navTriangle = triangles[index];
        toCheck.RemoveAt(0);
        connected.Add(index);
        foreach(int
        n
                in
        navTriangle.Neighbors)
        if (!toCheck.Contains(n) && !connected.Contains(n))
            toCheck.Add(n);
    }

#pragma endregion

#pragma region Fill holes and final iteration of NavTriangles

    vector<Vector3> fixedVertices = new vector<Vector3>();
    vector<int> fixedIndices = new vector<int>();

    foreach(NavMeshTriangle
    t
            in
    connected.Select(i = > triangles[i]))
    foreach(int
    tVertex
            in
    t.Vertices)
    {
        if (!fixedVertices.Contains(verts[tVertex]))
            fixedVertices.Add(verts[tVertex]);

        fixedIndices.Add(fixedVertices.IndexOf(verts[tVertex]));

        Vector2Int id = new Vector2Int((int) Math.Floor(verts[tVertex].x / groupSize),
                                       (int) Math.Floor(verts[tVertex].Z / groupSize));
        if (vertsByPosition.TryGetValue(id, out vector<int> outListA))
        {
            if (outListA.Contains(fixedVertices.IndexOf(verts[tVertex])))
                outListA.Add(fixedVertices.IndexOf(verts[tVertex]));
        }
        else
        {
            vertsByPosition.Add(id, new vector<int>{fixedVertices.IndexOf(verts[tVertex])});
        }
    }

    FillHoles(fixedVertices, fixedIndices);

    vector<NavMeshTriangle> fixedTriangles = new vector<NavMeshTriangle>();
    map<int, vector<int>> fixedTrianglesByVertexId = new map<int, vector<int>>();
    for (int i = 0; i < fixedVertices.size; i++)
        fixedTrianglesByVertexId.Add(i, new vector<int>());

    SetupNavTriangles(fixedIndices, fixedTriangles, fixedTrianglesByVertexId);

    fixedTriangles = SetupNeighbors(fixedTriangles, fixedTrianglesByVertexId);

    for (int i = 0; i < fixedTriangles.size; i++)
        fixedTriangles[i].SetBorderWidth(fixedVertices, fixedTriangles);

#pragma endregion

    NavMeshOptimized result = new NavMeshOptimized();
    result.SetValues(fixedVertices.ToArray(), fixedIndices.ToArray(), fixedTriangles.ToArray());
    return result;
}

void CheckOverlap(vector<Vector3> verts, vector<int> indices,
                  map<Vector2Int, vector<int>> vertsByPos,
                  float groupSize) {
    //The ids of vertices to be removed, grouped index.
    map<int, vector<int>> removed = *new map<int, vector<int>>();
    for (int currentVertIndex = 0; currentVertIndex < verts.size(); currentVertIndex++) {
        //Check if the current vertex id is part of the to be removed.
        if (removed.TryGetValue((int) MathF.Floor(currentVertIndex / groupSize), out vector<int> removedList))
        //If its to be removed then dont check this vertex.
        if (removedList.Contains(currentVertIndex))
            continue;

        //2D id of the vertex based on its x and z values and grouped by group size.
        Vector2Int id = new Vector2Int((int) MathF.Floor(verts[currentVertIndex].X / groupSize),
                                       (int) MathF.Floor(verts[currentVertIndex].Z / groupSize));

        //Get the
        vector<int> toCheck = *new vector<int>();
        for (int x = -1; x <= 1; x++)
            for (int y = -1; y <= 1; y++)
                if (vertsByPos.TryGetValue(id + new Vector2Int(x, y), out vector<int> list))
        toCheck.AddRange(list);

        toCheck = toCheck.Where(x = > x != currentVertIndex).ToList();

        for (const int &other: toCheck) {
            if (removed.TryGetValue((int) MathF.Floor(other / groupSize), out removedList))
            if (removedList.Contains(other))
                continue;

            if (Vector3.Distance(verts[currentVertIndex], verts[other]) > OVERLAP_CHECK_DISTANCE)
                continue;

            if (removed.TryGetValue((int) MathF.Floor(other / groupSize), out removedList))
            removedList.Add(other);
            else
            removed.Add((int) MathF.Floor(other / groupSize), new vector<int>{other});

            for (int indicesIndex = 0; indicesIndex < indices.Count; indicesIndex++)
                if (indices[indicesIndex] == other)
                    indices[indicesIndex] = currentVertIndex;
        }
    }

    vector<int> toRemove = removed.Values.SelectMany(l = > l).OrderBy(x = > -x).ToList();
    for (int i = 0; i < toRemove.Count; i++) {
        int index = toRemove[i];
        vertsByPos[
                new Vector2Int((int) MathF.Floor(verts[index].X / groupSize),
                               (int) MathF.Floor(verts[index].Z / groupSize))].Remove(index);
        verts.RemoveAt(index);

        for (int j = 0; j < indices.Count; j++)
            if (indices[j] >= index)
                indices[j] = indices[j] - 1;
    }

    for (int i = indices.Count - 1; i >= 0; i--) {
        if (i % 3 != 0)
            continue;

        if (indices[i] == indices[i + 1] || indices[i] == indices[i + 2] || indices[i + 1] == indices[i + 2] ||
            indices[i] >= verts.Count || indices[i + 1] >= verts.Count || indices[i + 2] >= verts.Count) {
            indices.RemoveAt(i);
            indices.RemoveAt(i);
            indices.RemoveAt(i);
        }
    }
}

void FillHoles(vector<Vector3> verts, vector<int> indices) {
    map<int, vector<int>> connectionsByIndex = new map<int, vector<int>>();
    map<int, vector<int>> indicesByIndex = new map<int, vector<int>>();
    for (int i = 0; i < verts.Count; i++) {
        connectionsByIndex.Add(i, new vector<int>());
        indicesByIndex.Add(i, new vector<int>());
    }

    for (int i = 0; i < indices.Count; i += 3) {
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

        int []
        arr = {indices[i], indices[i + 1], indices[i + 2]};
        indicesByIndex[indices[i]].AddRange(arr);
        indicesByIndex[indices[i + 1]].AddRange(arr);
        indicesByIndex[indices[i + 2]].AddRange(arr);
    }

    for (int i = 0; i < verts.Count; i++) {
        Vector2 p = verts[i].XZ();

        for (int j = 0; j < indices.Count; j += 3) {
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

    for (int original = 0; original < verts.Count; original++) {
        vector<int> originalConnections = connectionsByIndex[original];

        for (int otherIndex = 0; otherIndex < originalConnections.Count; otherIndex++) {
            int other = originalConnections[otherIndex];

            if (other <= original)
                continue;

            for (int finalIndex = otherIndex + 1; finalIndex < originalConnections.Count; finalIndex++) {
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

                for (int x = 0; x < indices.Count; x += 3) {
                    vector<int> checkArr = new vector<int>{indices[x], indices[x + 1], indices[x + 2]};
                    if (checkArr.Contains(original) && checkArr.Contains(other) && checkArr.Contains(final)) {
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
                        MathC.PointWithinTriangle2DWithTolerance(c, aP, bP, cP)) {
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

void SetupNavTriangles(vector<int> indices,
                       vector<NavMeshTriangle> triangles, map<int, vector<int>> trianglesByVertexID) {
    for (int i = 0; i < indices.Count; i += 3) {
        int a = indices[i], b = indices[i + 1], c = indices[i + 2];
        NavMeshTriangle triangle = new NavMeshTriangle(i / 3, a, b, c);

        triangles.Add(triangle);

        int tID = triangles.Count - 1;

        if (trianglesByVertexID.TryGetValue(a, out vector<int> list))
        {
            if (!list.Contains(tID))
                list.Add(tID);
        }
        else
        {
            trianglesByVertexID.Add(a, new vector<int>()
            { tID });
        }

        if (trianglesByVertexID.TryGetValue(b, out list))
        {
            if (!list.Contains(tID))
                list.Add(tID);
        }
        else
        {
            trianglesByVertexID.Add(b, new vector<int>()
            { tID });
        }

        if (trianglesByVertexID.TryGetValue(c, out list))
        {
            if (!list.Contains(tID))
                list.Add(tID);
        }
        else
        {
            trianglesByVertexID.Add(c, new vector<int>()
            { tID });
        }
    }
}

vector<NavMeshTriangle> SetupNeighbors(vector<NavMeshTriangle> triangles,
                                       map<int, vector<int>> trianglesByVertexID) {
    for (int i = 0; i < triangles.Count; i++) {
        vector<int> neighbors = new vector<int>();
        vector<int> possibleNeighbors = new vector<int>();
        possibleNeighbors.AddRange(trianglesByVertexID[triangles[i].Vertices[0]]);
        possibleNeighbors.AddRange(trianglesByVertexID[triangles[i].Vertices[1]]);
        possibleNeighbors.AddRange(trianglesByVertexID[triangles[i].Vertices[2]]);

        possibleNeighbors = possibleNeighbors.Where(t = > t != i).ToList();

        foreach(int
        t
                in
        possibleNeighbors)
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

vector<float> &XYZ(vector<float> *v) {
    return *new vector<float>{v->at(0), 0, v->at(1)};
}

vector<float> &XZ(vector<float> *vector) {
    return *new std::vector<float>{vector->at(0), vector->at(2)};
}

int indexOf(vector<vector<float>> *list, vector<float> *element) {
    for (int i = 0; i < list->size(); ++i) {
        if (list->at(i) == *element)
            return i;
    }

    return -1;
}


void insertRange(vector<int> *target, vector<int> *from) {
    for (int i = 0; i < from->size(); ++i) {
        target->push_back(from->at(i));
    }
}


vector<float> &offsetVector(vector<float> *v, int x, int y) {
    return *new vector<float>{v->at(0) + x, v->at(1) + y};
}

bool contains(vector<int> *v, int *target) {
    for (int i = 0; i < v->size(); ++i) {
        if (v->at(i) == *target)
            return true;
    }

    return false;
}

bool contains(vector<vector<float>> *v, vector<float> *target) {
    for (int i = 0; i < v->size(); ++i) {
        if (v->at(i) == *target)
            return true;
    }

    return false;
}

int main() {
    constexpr int averageCount = 1;

    const vector<string> file_letter = {"S", "M", "L"};

    const fs::path folder_path = fs::current_path().parent_path().parent_path() += "\\JsonFiles\\";
    cout << "Using json text files from folder:\n" << folder_path << "\n";

    for (int letter_index = 0; letter_index < 3; ++letter_index) {
        for (int number_index = 1; number_index <= 5; ++number_index) {

            cout << "Optimization for: " << file_letter.at(letter_index) << " " << number_index << '\n';

            fs::path fileName = fs::current_path().parent_path().parent_path() +=
                                        "\\JsonFiles\\" +
                                        file_letter.at(letter_index) +
                                        " " + to_string(number_index) +
                                        ".txt";

            NavMeshImport *navMeshImport = loadJsonToNavMeshImport(fileName);

            long long total_time = 0.0;

            for (int i = 0; i < averageCount; ++i) {

                cout << "Check " << to_string(i + 1) << "\n";

                cout << "Start vertex count: " << navMeshImport->get_vertices()->size() << "\n";
                cout << "Start indices count: " << navMeshImport->get_indices()->size() << "\n";

                vector<float> &cleanPoint = *new vector<float>;
                for (const float i: *navMeshImport->getCleanPoint())
                    cleanPoint.push_back(i);

                vector<vector<float>> &vertices = *new vector<vector<float>>;
                for (const vector<float> i: *navMeshImport->get_vertices())
                    vertices.push_back(i);

                vector<int> &indices = *new vector<int>;
                for (const int i: *navMeshImport->get_indices())
                    indices.push_back(i);

                auto timerStart = high_resolution_clock::now();

                NavMeshOptimized &navMeshOptimized = optimize_nav_mesh(cleanPoint, vertices, indices);

                auto timerEnd = high_resolution_clock::now();

                auto time = duration_cast<milliseconds>(timerEnd - timerStart);

                total_time += time.count();

                cout << "Final vertex count: " << navMeshOptimized.getVertices().size() << "\n";
                cout << "Final triangle count: " << navMeshOptimized.getTriangles().size() << "\n";
                cout << "Time: " << time.count() << "(ms)\n";
                cout << "Time: " << time.count() / 1000 << "(s)\n\n";
            }

            if (averageCount == 1)
                continue;

            cout << "Repeat count: " << averageCount << "\n";

            cout << "Total time for repeats: " << total_time << "(ms)\n";
            cout << "Total time for repeats: " << total_time / 1000 << "(s)\n";

            cout << "Average time for " << file_letter.at(letter_index) << " " << number_index << ": "
                 << total_time / averageCount << "(ms)\n";
            cout << "Average time for " << file_letter.at(letter_index) << " " << number_index << ": "
                 << total_time / averageCount / 1000 << "(s)\n\n";
        }

    }

    return 0;
}