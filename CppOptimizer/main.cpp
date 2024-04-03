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

void CheckOverlap(vector<Vector3> *verts, vector<int> *indices,
                  map<Vector2Int, vector<int>> *vertsByPosition, const float size);

void
SetupNavTriangles(vector<int> *indices, vector<NavMeshTriangle> *triangles, map<int, vector<int>> *trianglesByVertexId);

vector<NavMeshTriangle> SetupNeighbors(vector<NavMeshTriangle> *triangles, map<int, vector<int>> *trianglesByVertexId);

void FillHoles(vector<Vector3> *verts, vector<int> *indices);

bool contains(vector<int> *v, int *target);

bool contains(vector<Vector2> *v, Vector2 *target);

bool contains(vector<Vector3> *v, Vector3 *target);

vector<float> &offsetVector(Vector2 *v, int x, int y);

void insertRange(vector<int> *target, vector<int> *from);

int indexOf(vector<Vector2> *list, Vector2 *element);

int indexOf(vector<Vector3> *list, Vector3 *element);

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

NavMeshOptimized &OptimizeNavMesh(Vector3 *cleanPoint, vector<Vector3> *verts, vector<int> *indices) {
#pragma region Check Vertices and Indices for overlap

    map<Vector2Int, vector<int>> &vertsByPosition = *new map<Vector2Int, vector<int>>();

    const float groupSize = 5.0f;

    for (int i = 0; i < verts->size(); i++) {
        Vector3 &v = verts->at(i);
        Vector2Int &id = *new Vector2Int((int) floor(v.x / groupSize),
                                         (int) floor(v.z / groupSize));

        if (vertsByPosition.find(id) == vertsByPosition.end()) {
            vertsByPosition.insert({id, *new vector<int>});
        }

        vertsByPosition.at(id).push_back(i);
    }

    CheckOverlap(verts, indices, &vertsByPosition, groupSize);

#pragma endregion

#pragma region Create first iteration of NavTriangles

    vector<NavMeshTriangle> &triangles = *new vector<NavMeshTriangle>();
    map<int, vector<int>> &trianglesByVertexId = *new map<int, vector<int>>();
    for (int i = 0; i < verts->size(); i++)
        trianglesByVertexId.insert({i, *new vector<int>()});

    SetupNavTriangles(indices, &triangles, &trianglesByVertexId);

    triangles = SetupNeighbors(&triangles, &trianglesByVertexId);

#pragma endregion

#pragma region Check neighbor connections

    int closestVert = 0;
    float closestDistance = cleanPoint->QuickSquareDistance(verts->at(closestVert));

    for (int i = 1; i < verts->size(); i++) {
        float d = cleanPoint->QuickSquareDistance(verts->at(i));
        if (d >= closestDistance)
            continue;

        if (trianglesByVertexId.find(i) != trianglesByVertexId.end()) {
            bool found = false;
            for (const int &t: trianglesByVertexId.at(i))
                if (!triangles.at(t).neighbors().empty()) {
                    found = true;
                    break;
                }

            if (!found)
                continue;
        }

        closestDistance = d;
        closestVert = i;
    }

    vector<int> &connected = *new vector<int>(), &toCheck = *new vector<int>();
    toCheck.AddRange(trianglesByVertexId[closestVert]);

    while (toCheck.size() > 0) {
        int index = toCheck[0];
        NavMeshTriangle navTriangle = triangles[index];
        toCheck.erase(toCheck.begin());
        connected.push_back(index);

        for (int &n: navTriangle.neighbors()) {
            if (!contains(&toCheck, &n) && !contains(&connected, &n))
                toCheck.push_back(n);
        }
    }

#pragma endregion

#pragma region Fill holes and final iteration of NavTriangles

    vector<Vector3> &fixedVertices = *new vector<Vector3>();
    vector<int> &fixedIndices = *new vector<int>();

    for (const int &i: connected) {
        for (const int tVertex: triangles.at(i).vertices()) {
            if (!contains(&fixedVertices, &verts->at(tVertex)))
                fixedVertices.push_back(verts->at(tVertex));

            fixedIndices.push_back(indexOf(&fixedVertices, &verts->at(tVertex)));

            Vector2Int &id = *new Vector2Int((int) floor(verts->at(tVertex).x / groupSize),
                                             (int) floor(verts->at(tVertex).z / groupSize));

            if (vertsByPosition.find(id) == vertsByPosition.end())
                vertsByPosition.insert({id, *new vector<int>});

            vertsByPosition.at(id).push_back(indexOf(&fixedVertices, &verts->at(tVertex)));
        }
    }

    FillHoles(&fixedVertices, &fixedIndices);

    vector<NavMeshTriangle> fixedTriangles = *new vector<NavMeshTriangle>();
    map<int, vector<int>> fixedTrianglesByVertexId = *new map<int, vector<int>>();
    for (int i = 0; i < fixedVertices.size(); i++)
        fixedTrianglesByVertexId.insert({i, *new vector<int>()});

    SetupNavTriangles(&fixedIndices, &fixedTriangles, &fixedTrianglesByVertexId);

    SetupNeighbors(&fixedTriangles, &fixedTrianglesByVertexId);

    for (int i = 0; i < fixedTriangles.size(); i++)
        fixedTriangles.at(i).SetBorderWidth(&fixedVertices, &fixedTriangles);

#pragma endregion

    NavMeshOptimized &result = *new NavMeshOptimized();
    result.SetValues(&fixedVertices, &fixedTriangles, groupSize);
    return result;
}

void CheckOverlap(vector<Vector3> verts, vector<int> indices,
                  map<Vector2Int, vector<int>> vertsByPos,
                  float groupSize) {
    //The ids of vertices to be removed, grouped index.
    map<int, vector<int>> removed = *new map<int, vector<int>>();
    for (int currentVertIndex = 0; currentVertIndex < verts.size(); currentVertIndex++) {
        //Check if the current vertex id is part of the to be removed.
        if (removed.TryGetValue((int) floor(currentVertIndex / groupSize), out vector<int> removedList))
        //If its to be removed then dont check this vertex.
        if (removedList.Contains(currentVertIndex))
            continue;

        //2D id of the vertex based on its x and z values and grouped by group size.
        Vector2Int id = new Vector2Int((int) floor(verts[currentVertIndex].x / groupSize),
                                       (int) floor(verts[currentVertIndex].z / groupSize));

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

            for (int indicesIndex = 0; indicesIndex < indices.size; indicesIndex++)
                if (indices[indicesIndex] == other)
                    indices[indicesIndex] = currentVertIndex;
        }
    }

    vector<int> toRemove = removed.Values.SelectMany(l = > l).OrderBy(x = > -x).ToList();
    for (int i = 0; i < toRemove.size(); i++) {
        int index = toRemove[i];
        Vector2Int l = *new Vector2Int((int) floor(verts[index].x / groupSize),
                                       (int) floor(verts[index].z / groupSize));
        vertsByPos.at(l).Remove(index);
        verts.RemoveAt(index);

        for (int j = 0; j < indices.size(); j++)
            if (indices[j] >= index)
                indices[j] = indices[j] - 1;
    }

    for (int i = indices.size() - 1; i >= 0; i--) {
        if (i % 3 != 0)
            continue;

        if (indices[i] == indices[i + 1] || indices[i] == indices[i + 2] || indices[i + 1] == indices[i + 2] ||
            indices[i] >= verts.size() || indices[i + 1] >= verts.size() || indices[i + 2] >= verts.size()) {
            indices.RemoveAt(i);
            indices.RemoveAt(i);
            indices.RemoveAt(i);
        }
    }
}

void FillHoles(vector<Vector3> verts, vector<int> indices) {
    map<int, vector<int>> connectionsByIndex = *new map<int, vector<int>>();
    map<int, vector<int>> indicesByIndex = *new map<int, vector<int>>();
    for (int i = 0; i < verts.size(); i++) {
        connectionsByIndex.insert({i, *new vector<int>()});
        indicesByIndex.insert({i, *new vector<int>()});
    }

    for (int i = 0; i < indices.size(); i += 3) {
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

        vector<int> arr = *new vector<int>{indices[i], indices[i + 1], indices[i + 2]};
        indicesByIndex[indices[i]].AddRange(arr);
        indicesByIndex[indices[i + 1]].AddRange(arr);
        indicesByIndex[indices[i + 2]].AddRange(arr);
    }

    for (int i = 0; i < verts.size(); i++) {
        Vector2 p = verts[i].XZ();

        for (int j = 0; j < indices.size(); j += 3) {
            if (indices[j] == i || indices[j + 1] == i || indices[j + 2] == i)
                continue;

            Vector2 a = verts[indices[j]].XZ(), b = verts[indices[j + 1]].XZ(), c = verts[indices[j + 2]].XZ();

            if (!MathC::PointWithinTriangle2DWithTolerance(p, a, b, c))
                continue;

            Vector2 close1 = MathC::ClosetPointOnLine(p, a, b),
                    close2 = MathC::ClosetPointOnLine(p, a, c),
                    close3 = MathC::ClosetPointOnLine(p, b, b);

            Vector2 close = close3;
            if (Vector2::Distance(close1, p) < Vector2::Distance(close2, p) &&
                Vector2::Distance(close1, p) < Vector2::Distance(close3, p))
                close = close1;
            else if (Vector2::Distance(close2, p) < Vector2::Distance(close3, p))
                close = close2;

            Vector2 offset = close - p;

            verts[i] += (offset.Normalize() * (MathC::Magnitude(offset) + .01f)).ToV3(0);
        }
    }

    for (int original = 0; original < verts.size(); original++) {
        vector<int> originalConnections = connectionsByIndex[original];

        for (int otherIndex = 0; otherIndex < originalConnections.size(); otherIndex++) {
            int other = originalConnections[otherIndex];

            if (other <= original)
                continue;

            for (int finalIndex = otherIndex + 1; finalIndex < originalConnections.size(); finalIndex++) {
                int final = originalConnections[finalIndex];

                if (final <= original || !connectionsByIndex[final].Contains(other))
                    continue;

                bool denied = false;

                Vector2 a = XZ(verts[original]), b = XZ(verts[other]), c = XZ(verts[final]);
                Vector2 center = Vector2.Lerp(Vector2.Lerp(a, b, .5f), c, .5f);

                float minX = MathC::Min(MathC::Min(a.x, b.x), c.x),
                        minY = MathC::Min(MathC::Min(a.y, b.y), c.y),
                        maxX = MathC::Max(MathC::Max(a.x, b.x), c.x),
                        maxY = MathC::Max(MathC::Max(a.y, b.y), c.y);

                for (int x = 0; x < indices.size(); x += 3) {
                    vector<int> checkArr = *new vector<int>{indices[x], indices[x + 1], indices[x + 2]};
                    if (checkArr.Contains(original) && checkArr.Contains(other) && checkArr.Contains(final)) {
                        //The triangle already exists
                        denied = true;
                        break;
                    }

                    Vector2 aP = XZ(verts[checkArr[0]]),
                            bP = XZ(verts[checkArr[1]]),
                            cP = XZ(verts[checkArr[2]]);

                    //Bounding
                    if (maxX < MathC::Min(MathC.Min(aP.x, bP.x), cP.x) ||
                        maxY < MathC::Min(MathC.Min(aP.y, bP.y), cP.y) ||
                        minX > MathC::Max(MathC.Max(aP.x, bP.x), cP.x) ||
                        minY > MathC::Max(MathC.Max(aP.y, bP.y), cP.y))
                        continue;

                    //One of the new triangle points is within an already existing triangle
                    if (MathC.PointWithinTriangle2DWithTolerance(center, aP, bP, cP) ||
                        MathC.PointWithinTriangle2DWithTolerance(a, aP, bP, cP) ||
                        MathC.PointWithinTriangle2DWithTolerance(b, aP, bP, cP) ||
                        MathC.PointWithinTriangle2DWithTolerance(c, aP, bP, cP)) {
                        denied = true;
                        break;
                    }

                    if (!MathC::TriangleIntersect2D(a, b, c, aP, bP, cP))
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
    for (int i = 0; i < indices.size(); i += 3) {
        int a = indices[i], b = indices[i + 1], c = indices[i + 2];
        NavMeshTriangle triangle = new NavMeshTriangle(i / 3, a, b, c);

        triangles.Add(triangle);

        int tID = triangles.size() - 1;

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
    for (int i = 0; i < triangles.size(); i++) {
        vector<int> neighbors = *new vector<int>();
        vector<int> possibleNeighbors = *new vector<int>();
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

            if (triangles.size == 3)
                break;
        }

        triangles[i].SetNeighborIDs(neighbors.ToArray());
    }

    return triangles;
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

                NavMeshOptimized &navMeshOptimized = OptimizeNavMesh(cleanPoint, vertices, indices);

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