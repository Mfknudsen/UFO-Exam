#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include <map>
#include <algorithm>

using json = nlohmann::json;

using namespace std;
using namespace chrono;

namespace fs = filesystem;

#include "NavMeshImport.h"
#include "NavMeshOptimized.h"
#include "MathC.h"
#include "Vector2Int.h"
#include "Vector3.h"
#include "OptimizedResult.h"

void CheckOverlap(vector<Vector3> &verts, vector<int> &indices,
                  map<Vector2Int, vector<int>> &vertsByPosition, float size);

void
SetupNavTriangles(const vector<int> &indices, vector<NavMeshTriangle> &triangles,
                  map<int, vector<int>> &trianglesByVertexId);

void SetupNeighbors(vector<NavMeshTriangle> &triangles, map<int, vector<int>> &trianglesByVertexId);

void FillHoles(vector<Vector3> &verts, vector<int> &indices);

vector<int> SharedBetween(const vector<int> &v1, const vector<int> &v2);

void writeCsv(fs::path &fileName, OptimizedResult &r);

NavMeshImport loadJsonToNavMeshImport(fs::path &file) {
    cout << "   Importing navigation mesh from file:" << "\n";
    cout << "   " << file << "\n";

    ifstream str(file);
    json js = json::parse(str);

    vector<float> cleanPoint = vector<float>{(float) js["cleanPoint"]["x"],
                                             (float) js["cleanPoint"]["y"],
                                             (float) js["cleanPoint"]["z"]};
    cout << "   Clean Point {" << (float) cleanPoint[0] << " , " << (float) cleanPoint[1] << " , "
         << (float) cleanPoint[2] << "}" << "\n";

    vector<Vector3> vertexPoints = vector<Vector3>();

    for (int i = 0; i < (int) js["x"].size(); ++i) {
        vertexPoints.emplace_back((float) js["x"][i], (float) js["y"][i], (float) js["z"][i]);
    }

    cout << "   Vertex count: " << vertexPoints.size() << "\n";

    vector<int> indices = vector<int>();

    for (const auto &item: js["indices"])
        indices.push_back((int) item);

    cout << "   Indices count: " << indices.size() << "\n\n";

    return {cleanPoint, vertexPoints, indices,
            (int) js["finalVertexCount"],
            (int) js["finalIndicesCount"],
            (int) js["finalTriangleCount"]};
}

NavMeshOptimized OptimizeNavMesh(const Vector3 cleanPoint, vector<Vector3> &verts, vector<int> &indices) {
#pragma region Check Vertices and Indices for overlap

    map<Vector2Int, vector<int>> vertsByPosition = map<Vector2Int, vector<int>>();

    const float groupSize = 5.0f;

    for (int i = 0; i < (int) verts.size(); i++) {
        Vector3 &v = verts[i];
        Vector2Int id = Vector2Int((int) floor(v.x / groupSize),
                                   (int) floor(v.z / groupSize));

        if (vertsByPosition.find(id) == vertsByPosition.end()) {
            vertsByPosition.insert({id, vector<int>()});
        }

        vertsByPosition[id].push_back(i);
    }

    CheckOverlap(verts, indices, vertsByPosition, groupSize);

#pragma endregion

#pragma region Create first iteration of NavTriangles

    vector<NavMeshTriangle> triangles = vector<NavMeshTriangle>();
    map<int, vector<int>> trianglesByVertexId = map<int, vector<int>>();
    for (int i = 0; i < (int) verts.size(); i++)
        trianglesByVertexId.insert({i, vector<int>()});

    SetupNavTriangles(indices, triangles, trianglesByVertexId);

    SetupNeighbors(triangles, trianglesByVertexId);

#pragma endregion

#pragma region Check neighbor connections

    int closestVert = 0;
    float closestDistance = Vector3::Distance(cleanPoint, verts[closestVert]);

    for (int i = 1; i < (int) verts.size(); i++) {
        const float d = Vector3::Distance(cleanPoint, verts[i]);

        if (d >= closestDistance)
            continue;

        if (trianglesByVertexId.find(i) != trianglesByVertexId.end()) {
            bool found = false;
            for (const int &t: trianglesByVertexId[i])
                if (!triangles[t].neighbors().empty()) {
                    found = true;
                    break;
                }

            if (!found)
                continue;
        }

        closestDistance = d;
        closestVert = i;
    }

    vector<int> connected = vector<int>(), toCheck = vector<int>();
    toCheck.insert(toCheck.end(), trianglesByVertexId[closestVert].begin(), trianglesByVertexId[closestVert].end());

    while (!toCheck.empty()) {
        int index = toCheck[0];
        NavMeshTriangle navTriangle = triangles[index];
        toCheck.erase(toCheck.begin());
        connected.push_back(index);

        for (int &n: navTriangle.neighbors()) {
            if (find(toCheck.begin(), toCheck.end(), n) == toCheck.end() &&
                find(connected.begin(), connected.end(), n) == connected.end())
                toCheck.push_back(n);
        }
    }

#pragma endregion

#pragma region Fill holes and final iteration of NavTriangles

    vector<Vector3> fixedVertices = vector<Vector3>();
    vector<int> fixedIndices = vector<int>();

    for (const int &i: connected) {
        for (const int tVertex: triangles[i].vertices()) {
            if (find(fixedVertices.begin(), fixedVertices.end(), verts[tVertex]) == fixedVertices.end())
                fixedVertices.push_back(verts[tVertex]);

            int elementIndex = (int) distance(fixedVertices.begin(),
                                              std::find(fixedVertices.begin(), fixedVertices.end(), verts[tVertex]));
            fixedIndices.push_back(elementIndex);

            Vector2Int id = Vector2Int((int) floor(verts[tVertex].x / groupSize),
                                       (int) floor(verts[tVertex].z / groupSize));

            if (vertsByPosition.find(id) == vertsByPosition.end())
                vertsByPosition.insert({id, vector<int>()});

            vertsByPosition[id].push_back(elementIndex);
        }
    }

    FillHoles(fixedVertices, fixedIndices);

    vector<NavMeshTriangle> fixedTriangles = vector<NavMeshTriangle>();
    map<int, vector<int>> fixedTrianglesByVertexId = map<int, vector<int>>();
    for (int i = 0; i < (int) fixedVertices.size(); i++)
        fixedTrianglesByVertexId.insert({i, vector<int>()});

    SetupNavTriangles(fixedIndices, fixedTriangles, fixedTrianglesByVertexId);

    SetupNeighbors(fixedTriangles, fixedTrianglesByVertexId);

    for (int i = 0; i < (int) fixedTriangles.size(); i++)
        fixedTriangles[i].SetBorderWidth(fixedVertices, fixedTriangles);

#pragma endregion

    NavMeshOptimized result = NavMeshOptimized();
    result.SetValues(fixedVertices, fixedIndices, fixedTriangles, groupSize);
    return result;
}

void CheckOverlap(vector<Vector3> &verts, vector<int> &indices,
                  map<Vector2Int, vector<int>> &vertsByPos,
                  const float groupSize) {
    const float overlapCheckDistance = 0.3f;
    map<int, vector<int>> removed = map<int, vector<int>>();

    for (int currentVertIndex = 0; currentVertIndex < (int) verts.size(); currentVertIndex++) {

        int iFloor = (int) floor((float) currentVertIndex / groupSize);
        if (removed.find(iFloor) != removed.end()) {
            if (find(removed[iFloor].begin(), removed[iFloor].end(), currentVertIndex) != removed[iFloor].end())
                continue;
        }

        //2D id of the vertex based on its x and z values and grouped by group size.
        Vector2Int id = Vector2Int((int) floor(verts[currentVertIndex].x / groupSize),
                                   (int) floor(verts[currentVertIndex].z / groupSize));

        vector<int> toCheck = vector<int>();
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                Vector2Int d = Vector2Int(id.x + x, id.y + y);
                if (vertsByPos.find(d) != vertsByPos.end())
                    toCheck.insert(toCheck.end(), vertsByPos[d].begin(), vertsByPos[d].end());
            }
        }

        for (const int &other: toCheck) {
            if (other == currentVertIndex)
                continue;

            int i = (int) floor((float) other / groupSize);

            if (removed.find(i) != removed.end())
                if (find(removed[i].begin(), removed[i].end(), other) != removed[i].end())
                    continue;

            if (Vector3::Distance(verts[currentVertIndex], verts[other]) > overlapCheckDistance)
                continue;

            if (removed.find(i) == removed.end()) {
                removed.insert({i, vector<int>()});
            }


            removed[i].push_back(other);

            for (int &index: indices)
                if (index == other)
                    index = currentVertIndex;
        }
    }

    vector<int> toRemove = vector<int>();
    for (const pair<const int, vector<int>> &pair: removed) {
        toRemove.insert(toRemove.end(), removed[pair.first].begin(), removed[pair.first].end());
    }

    sort(toRemove.begin(), toRemove.end(), greater<int>());

    for (const int &index: toRemove) {
        Vector2Int l = Vector2Int((int) floor(verts[index].x / groupSize),
                                  (int) floor(verts[index].z / groupSize));

        vector<int> by = vertsByPos[l];
        for (int j = (int) by.size() - 1; j >= 0; --j) {
            if (by[j] == index)
                by.erase(by.begin() + j);
        }

        verts.erase(verts.begin() + index);

        for (int &i: indices)
            if (i >= index)
                i = i - 1;
    }

    for (int i = (int) indices.size() - 3; i >= 0; i -= 3) {
        if (indices[i] == indices[i + 1] || indices[i] == indices[i + 2] ||
            indices[i + 1] == indices[i + 2] ||
            indices[i] >= (int) verts.size() || indices[i + 1] >= (int) verts.size() ||
            indices[i + 2] >= (int) verts.size()) {

            indices.erase(indices.begin() + i);
            indices.erase(indices.begin() + i);
            indices.erase(indices.begin() + i);
        }
    }
}

void FillHoles(vector<Vector3> &verts, vector<int> &indices) {
    vector<vector<int>> connectionsByIndex = vector<vector<int>>();
    connectionsByIndex.reserve(verts.size());
    vector<vector<int>> indicesByIndex = vector<vector<int>>();
    indicesByIndex.reserve(verts.size());
    for (int i = 0; i < (int) verts.size(); i++) {
        connectionsByIndex.emplace_back(8);
        indicesByIndex.emplace_back(8);
    }

    for (int i = 0; i < (int) indices.size(); i += 3) {

        vector<int> check = connectionsByIndex[indices[i]];
        if (find(check.begin(), check.end(), indices[i + 1]) == check.end())
            connectionsByIndex[indices[i]].push_back(indices[i + 1]);
        if (find(check.begin(), check.end(), indices[i + 2]) == check.end())
            connectionsByIndex[indices[i]].push_back(indices[i + 2]);

        check = connectionsByIndex[indices[i + 1]];
        if (find(check.begin(), check.end(), indices[i]) == check.end())
            connectionsByIndex[indices[i + 1]].push_back(indices[i]);
        if (find(check.begin(), check.end(), indices[i + 2]) == check.end())
            connectionsByIndex[indices[i + 1]].push_back(indices[i + 2]);

        check = connectionsByIndex[indices[i + 2]];
        if (find(check.begin(), check.end(), indices[i + 1]) == check.end())
            connectionsByIndex[indices[i + 2]].push_back(indices[i + 1]);
        if (find(check.begin(), check.end(), indices[i]) == check.end())
            connectionsByIndex[indices[i + 2]].push_back(indices[i]);

        vector<int> arr{indices[i], indices[i + 1], indices[i + 2]};

        connectionsByIndex[indices[i]].insert(connectionsByIndex[indices[i]].end(), arr.begin(), arr.end());
        connectionsByIndex[indices[i + 1]].insert(connectionsByIndex[indices[i + 1]].end(), arr.begin(), arr.end());
        connectionsByIndex[indices[i + 2]].insert(connectionsByIndex[indices[i + 2]].end(), arr.begin(), arr.end());
    }


    for (int i = 0; i < (int) verts.size(); i++) {
        Vector2 p = MathC::XZ(verts[i]);

        for (int j = 0; j < (int) indices.size(); j += 3) {
            if (indices[j] == i || indices[j + 1] == i || indices[j + 2] == i)
                continue;

            Vector2 a = MathC::XZ(verts[indices[j]]),
                    b = MathC::XZ(verts[indices[j + 1]]),
                    c = MathC::XZ(verts[indices[j + 2]]);

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
            float mag = offset.Magnitude() + 0.01f;
            offset.NormalizeSelf();
            Vector2 moved = offset * mag;
            Vector3 o = MathC::XYZ(moved);
            verts[i] = verts[i] + o;
        }
    }

    for (int original = 0; original < (int) verts.size(); original++) {
        vector<int> originalConnections = connectionsByIndex[original];
        int s = (int) originalConnections.size();

        for (int otherIndex = 0; otherIndex < s; otherIndex++) {
            int other = originalConnections[otherIndex];

            if (other <= original)
                continue;

            for (int finalIndex = otherIndex + 1; finalIndex < s; finalIndex++) {
                int final = originalConnections[finalIndex];

                if (final <= original || final <= other)
                    continue;

                vector<int> v = connectionsByIndex[final];
                if (find(v.begin(), v.end(), other) == v.end())
                    continue;

                bool denied = false;

                Vector2 a = MathC::XZ(verts[original]),
                        b = MathC::XZ(verts[other]),
                        c = MathC::XZ(verts[final]);

                Vector2 center = Vector2::Lerp(Vector2::Lerp(a, b, .5f), c, .5f);

                float minX = MathC::Min(MathC::Min(a.x, b.x), c.x),
                        minY = MathC::Min(MathC::Min(a.y, b.y), c.y),
                        maxX = MathC::Max(MathC::Max(a.x, b.x), c.x),
                        maxY = MathC::Max(MathC::Max(a.y, b.y), c.y);

                for (int x = 0; x < (int) indices.size(); x += 3) {
                    array checkArr = {indices[x], indices[x + 1], indices[x + 2]};


                    if ((checkArr[0] == original || checkArr[1] == original || checkArr[2] == original) &&
                        (checkArr[0] == other || checkArr[1] == other || checkArr[2] == other) &&
                        (checkArr[0] == final || checkArr[1] == final || checkArr[2] == final)) {
                        //The triangle already exists
                        denied = true;
                        break;
                    }

                    Vector2 aP = MathC::XZ(verts[checkArr[0]]),
                            bP = MathC::XZ(verts[checkArr[1]]),
                            cP = MathC::XZ(verts[checkArr[2]]);

                    //Bounding
                    if (maxX < MathC::Min(MathC::Min(aP.x, bP.x), cP.x))
                        continue;
                    if (maxY < MathC::Min(MathC::Min(aP.y, bP.y), cP.y))
                        continue;
                    if (minX > MathC::Max(MathC::Max(aP.x, bP.x), cP.x))
                        continue;
                    if (minY > MathC::Max(MathC::Max(aP.y, bP.y), cP.y))
                        continue;

                    //One of the new triangle points is within an already existing triangle
                    if (MathC::PointWithinTriangle2DWithTolerance(center, aP, bP, cP)) {
                        denied = true;
                        break;
                    }
                    if (MathC::PointWithinTriangle2DWithTolerance(a, aP, bP, cP)) {
                        denied = true;
                        break;
                    }
                    if (MathC::PointWithinTriangle2DWithTolerance(b, aP, bP, cP)) {
                        denied = true;
                        break;
                    }
                    if (MathC::PointWithinTriangle2DWithTolerance(c, aP, bP, cP)) {
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

                array arr = {original, other, final};
                indices.insert(indices.end(), arr.begin(), arr.end());
            }
        }
    }
}

void SetupNavTriangles(const vector<int> &indices, vector<NavMeshTriangle> &triangles,
                       map<int, vector<int>> &trianglesByVertexID) {
    for (int i = 0; i < (int) indices.size(); i += 3) {
        int a = indices[i], b = indices[i + 1], c = indices[i + 2];
        NavMeshTriangle triangle = NavMeshTriangle(i / 3, a, b, c);

        triangles.push_back(triangle);

        int tID = (int) triangles.size() - 1;

        if (trianglesByVertexID.find(a) == trianglesByVertexID.end())
            trianglesByVertexID.insert({a, vector<int>()});

        if (trianglesByVertexID.find(b) == trianglesByVertexID.end())
            trianglesByVertexID.insert({b, vector<int>()});

        if (trianglesByVertexID.find(c) == trianglesByVertexID.end())
            trianglesByVertexID.insert({c, vector<int>()});

        trianglesByVertexID[a].push_back(tID);
        trianglesByVertexID[b].push_back(tID);
        trianglesByVertexID[c].push_back(tID);
    }
}

void SetupNeighbors(vector<NavMeshTriangle> &triangles,
                    map<int, vector<int>> &trianglesByVertexID) {
    for (int i = 0; i < (int) triangles.size(); i++) {
        vector<int> neighbors = vector<int>();
        vector<int> possibleNeighbors = vector<int>();

        possibleNeighbors.insert(possibleNeighbors.end(), trianglesByVertexID[triangles[i].vertices()[0]].begin(),
                                 trianglesByVertexID[triangles[i].vertices()[0]].end());
        possibleNeighbors.insert(possibleNeighbors.end(), trianglesByVertexID[triangles[i].vertices()[1]].begin(),
                                 trianglesByVertexID[triangles[i].vertices()[1]].end());
        possibleNeighbors.insert(possibleNeighbors.end(), trianglesByVertexID[triangles[i].vertices()[2]].begin(),
                                 trianglesByVertexID[triangles[i].vertices()[2]].end());

        for (const int &t: possibleNeighbors) {
            if (t == i)
                continue;

            if (find(neighbors.begin(), neighbors.end(), t) != neighbors.end())
                continue;

            if (SharedBetween(triangles[i].vertices(), triangles[t].vertices()).size() == 2)
                neighbors.push_back(t);
        }

        triangles[i].SetNeighborIds(neighbors);
    }
}

vector<int> SharedBetween(const vector<int> &v1, const vector<int> &v2) {
    vector<int> result = vector<int>();
    result.reserve(2);
    for (const auto &item1: v1) {
        for (const auto &item2: v2) {
            if (item1 == item2) {
                result.push_back(item1);
                continue;
            }
        }
    }

    return result;
}


int main() {
    cout << setprecision(8);
    const int averageCount = 10;

    const vector<string> file_letter = {"S", "M", "L"};

    const fs::path folder_path = fs::current_path().parent_path().parent_path() += "\\JsonFiles\\";
    cout << "Using json text files from folder:\n" << folder_path << "\n";

    for (int letter_index = 0; letter_index < 3; ++letter_index) {
        for (int number_index = 1; number_index <= 5; ++number_index) {

            cout << "Optimization for: " << file_letter[letter_index] << " " << number_index << '\n';

            fs::path fileName = fs::current_path().parent_path().parent_path() +=
                                        "\\JsonFiles\\" +
                                        file_letter[letter_index] +
                                        " " + to_string(number_index) +
                                        ".txt";

            NavMeshImport navMeshImport = loadJsonToNavMeshImport(fileName);

            long long total_time = 0.0;
            OptimizedResult allOptimized = OptimizedResult(averageCount);

            for (int i = 0; i < averageCount; ++i) {

                cout << "Check " << to_string(i + 1) << "\n";

                cout << "Start vertex count: " << navMeshImport.getVertices().size() << "\n";
                cout << "Start indices count: " << navMeshImport.getIndices().size() << "\n\n";


                const Vector3 cleanPoint = Vector3(navMeshImport.getCleanPoint()[0],
                                                   navMeshImport.getCleanPoint()[1],
                                                   navMeshImport.getCleanPoint()[2]);

                vector<Vector3> vertices = vector<Vector3>();
                for (Vector3 vertex: navMeshImport.getVertices()) {
                    vertices.emplace_back(vertex.x, vertex.y, vertex.z);
                }

                vector<int> indices = vector<int>();
                for (int index: navMeshImport.getIndices())
                    indices.push_back(index);

                auto timerStart = high_resolution_clock::now();

                NavMeshOptimized navMeshOptimized = OptimizeNavMesh(cleanPoint, vertices, indices);

                auto timerEnd = high_resolution_clock::now();

                auto time = duration_cast<milliseconds>(timerEnd - timerStart);

                allOptimized.vertexCount += (int) navMeshOptimized.getVertices().size();
                allOptimized.indicesCount += (int) navMeshOptimized.getIndices().size();
                allOptimized.triangleCount += (int) navMeshOptimized.getTriangles().size();

                total_time += time.count();

                cout << "Vertex count match: "
                     << (((int) navMeshOptimized.getVertices().size()) == navMeshImport.FV())
                     << " | " << navMeshImport.FV() - ((int) navMeshOptimized.getVertices().size()) << "\n";
                cout << "Indices count match: "
                     << (((int) navMeshOptimized.getIndices().size()) == navMeshImport.FI())
                     << " | " << navMeshImport.FI() - ((int) navMeshOptimized.getIndices().size()) << "\n";
                cout << "Triangle count match: "
                     << (((int) navMeshOptimized.getTriangles().size()) == navMeshImport.FT())
                     << " | " << navMeshImport.FT() - ((int) navMeshOptimized.getTriangles().size()) << "\n\n";

                cout << "Final vertex count: " << navMeshOptimized.getVertices().size() << "\n";
                cout << "Final indices count: " << navMeshOptimized.getIndices().size() << "\n";
                cout << "Final triangle count: " << navMeshOptimized.getTriangles().size() << "\n";
                cout << "Time: " << time.count() << "(ms)\n";
                cout << "Time: " << (float) time.count() / 1000.0f << "(s)\n\n";
            }

            if (averageCount == 1)
                continue;

            cout << "Repeat count: " << averageCount << "\n";

            cout << "Total time for repeats: " << total_time << "(ms)\n";
            cout << "Total time for repeats: " << total_time / 1000 << "(s)\n";

            cout << "Average time for " << file_letter[letter_index] << " " << number_index << ": "
                 << total_time / averageCount << "(ms)\n";
            cout << "Average time for " << file_letter[letter_index] << " " << number_index << ": "
                 << (float) total_time / (float) averageCount / 1000.0f << "(s)\n\n";

            allOptimized.totalTime = (float) total_time;
            allOptimized.averageTime = (float) total_time / (float) averageCount;
            allOptimized.vertexCount /= averageCount;
            allOptimized.indicesCount /= averageCount;
            allOptimized.triangleCount /= averageCount;

            fileName = fs::current_path().parent_path().parent_path() +=
                    "\\CppResults\\" +
                    file_letter[letter_index] +
                    " " + to_string(number_index) +
                    ".csv";

            writeCsv(fileName, allOptimized);
        }
    }

    return 0;
}

void writeCsv(fs::path &fileName, OptimizedResult &r) {
    ofstream file(fileName);
    file << "AverageCount,VertexCount,IndicesCount,TriangleCount,TotalTime,AverageTime" << endl;
    file << r.averageCount << "," << r.vertexCount << "," << r.indicesCount << "," << r.triangleCount << ","
         << r.totalTime << "," << r.averageTime << endl;
    file.close();
}
