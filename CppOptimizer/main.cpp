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

void CheckOverlap(vector<Vector3> &verts, vector<int> &indices,
                  map<Vector2Int, vector<int>> &vertsByPosition, float size);

void
SetupNavTriangles(vector<int> &indices, vector<NavMeshTriangle> &triangles, map<int, vector<int>> &trianglesByVertexId);

void SetupNeighbors(vector<NavMeshTriangle> &triangles, map<int, vector<int>> &trianglesByVertexId);

void FillHoles(vector<Vector3> &verts, vector<int> &indices);

bool contains(vector<int> &v, int &target);

bool contains(vector<Vector3> &v, Vector3 &target);

void insertRange(vector<int> &target, vector<int> &from);

int indexOf(vector<Vector3> &list, Vector3 &element);

bool hasKey(map<Vector2Int, vector<int>> &mapTarget, Vector2Int &key);

bool hasKey(map<int, vector<int>> &mapTarget, int &key);

vector<int> &SharedBetween(vector<int> &v1, vector<int> &v2);

NavMeshImport *loadJsonToNavMeshImport(fs::path &file) {
    cout << "   Importing navigation mesh from file:" << "\n";
    cout << "   " << file << "\n";

    ifstream str(file);
    json js = json::parse(str);

    vector<float> &cleanPoint = *new vector<float>{(float) js["cleanPoint"]["x"],
                                                   (float) js["cleanPoint"]["y"],
                                                   (float) js["cleanPoint"]["z"]};
    cout << "   Clean Point {" << cleanPoint[0] << " , " << cleanPoint[1] << " , " << cleanPoint[2] << "}" << "\n";

    vector<vector<float>> &vertexPoints = *new vector<vector<float>>;

    for (int i = 0; i < js["x"].size(); ++i) {
        vertexPoints.push_back(vector<float>{(float) js["x"][i], (float) js["y"][i], (float) js["z"][i]});
    }

    cout << "   Vertex count: " << vertexPoints.size() << "\n";

    vector<int> &indices = *new vector<int>;

    for (const auto &item: js["indices"])
        indices.push_back((int) item);

    cout << "   Indices count: " << indices.size() << "\n\n";

    return new NavMeshImport(cleanPoint, vertexPoints, indices,
                             (int) js["finalVertexCount"],
                             (int) js["finalIndicesCount"],
                             (int) js["finalTriangleCount"]);
}

NavMeshOptimized &OptimizeNavMesh(Vector3 &cleanPoint, vector<Vector3> &verts, vector<int> &indices) {
#pragma region Check Vertices and Indices for overlap

    map<Vector2Int, vector<int>> &vertsByPosition = *new map<Vector2Int, vector<int>>();

    const float groupSize = 5.0f;

    for (int i = 0; i < verts.size(); i++) {
        Vector3 &v = verts.at(i);
        Vector2Int &id = *new Vector2Int((int) floor(v.x / groupSize),
                                         (int) floor(v.z / groupSize));

        if (!hasKey(vertsByPosition, id)) {
            vertsByPosition.insert({id, *new vector<int>});
        }

        vertsByPosition.at(id).push_back(i);
    }
    cout << verts.size() << "\n";
    CheckOverlap(verts, indices, vertsByPosition, groupSize);
    cout << verts.size() << "\n";

#pragma endregion

#pragma region Create first iteration of NavTriangles

    vector<NavMeshTriangle> &triangles = *new vector<NavMeshTriangle>();
    map<int, vector<int>> &trianglesByVertexId = *new map<int, vector<int>>();
    for (int i = 0; i < verts.size(); i++)
        trianglesByVertexId.insert({i, *new vector<int>()});

    SetupNavTriangles(indices, triangles, trianglesByVertexId);

    SetupNeighbors(triangles, trianglesByVertexId);

#pragma endregion

#pragma region Check neighbor connections

    int closestVert = 0;
    float closestDistance = cleanPoint.QuickSquareDistance(verts.at(closestVert));

    for (int i = 1; i < verts.size(); i++) {
        const float d = cleanPoint.QuickSquareDistance(verts.at(i));

        if(i == 474)
        cout << d << "\n";
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

    cout << "Closest: " << closestVert << "\n";

    vector<int> &connected = *new vector<int>(), &toCheck = *new vector<int>();
    insertRange(toCheck, trianglesByVertexId.at(closestVert));

    while (!toCheck.empty()) {
        int index = toCheck[0];
        NavMeshTriangle navTriangle = triangles[index];
        toCheck.erase(toCheck.begin());
        connected.push_back(index);

        for (int &n: navTriangle.neighbors()) {
            if (!contains(toCheck, n) && !contains(connected, n))
                toCheck.push_back(n);
        }
    }

#pragma endregion

#pragma region Fill holes and final iteration of NavTriangles

    vector<Vector3> &fixedVertices = *new vector<Vector3>();
    vector<int> &fixedIndices = *new vector<int>();

    for (const int &i: connected) {
        for (const int tVertex: triangles.at(i).vertices()) {
            if (!contains(fixedVertices, verts.at(tVertex)))
                fixedVertices.push_back(verts.at(tVertex));

            fixedIndices.push_back(indexOf(fixedVertices, verts.at(tVertex)));

            Vector2Int &id = *new Vector2Int((int) floor(verts.at(tVertex).x / groupSize),
                                             (int) floor(verts.at(tVertex).z / groupSize));

            if (vertsByPosition.find(id) == vertsByPosition.end())
                vertsByPosition.insert({id, *new vector<int>});

            vertsByPosition.at(id).push_back(indexOf(fixedVertices, verts.at(tVertex)));
        }
    }

    FillHoles(fixedVertices, fixedIndices);

    vector<NavMeshTriangle> &fixedTriangles = *new vector<NavMeshTriangle>();
    map<int, vector<int>> &fixedTrianglesByVertexId = *new map<int, vector<int>>();
    for (int i = 0; i < fixedVertices.size(); i++)
        fixedTrianglesByVertexId.insert({i, *new vector<int>()});

    SetupNavTriangles(fixedIndices, fixedTriangles, fixedTrianglesByVertexId);

    SetupNeighbors(fixedTriangles, fixedTrianglesByVertexId);

    for (int i = 0; i < fixedTriangles.size(); i++)
        fixedTriangles.at(i).SetBorderWidth(&fixedVertices, &fixedTriangles);

#pragma endregion

    NavMeshOptimized &result = *new NavMeshOptimized();
    result.SetValues(&fixedVertices, &fixedIndices, &fixedTriangles, groupSize);
    return result;
}

void CheckOverlap(vector<Vector3> &verts, vector<int> &indices,
                  map<Vector2Int, vector<int>> &vertsByPos,
                  const float groupSize) {
    const float overlapCheckDistance = 0.3f;
    map<int, vector<int>> &removed = *new map<int, vector<int>>();

    for (int currentVertIndex = 0; currentVertIndex < verts.size(); currentVertIndex++) {

        int iFloor = (int) floor((float) currentVertIndex / groupSize);
        if (hasKey(removed, iFloor)) {
            if (contains(removed.at(iFloor), currentVertIndex))
                continue;
        }

        //2D id of the vertex based on its x and z values and grouped by group size.
        Vector2Int id = *new Vector2Int((int) floor(verts.at(currentVertIndex).x / groupSize),
                                        (int) floor(verts.at(currentVertIndex).z / groupSize));

        vector<int> toCheck = *new vector<int>();
        for (int x = -1; x <= 1; x++) {
            for (int y = -1; y <= 1; y++) {
                Vector2Int d = *new Vector2Int(id.x + x, id.y + y);
                if (hasKey(vertsByPos, d))
                    insertRange(toCheck, vertsByPos.at(d));
            }
        }

        for (int &other: toCheck) {
            if (other == currentVertIndex)
                continue;

            int i = (int) floor((float) other / groupSize);

            if (removed.find(i) != removed.end())
                if (contains(removed.at(i), other))
                    continue;

            if (Vector3::Distance(verts.at(currentVertIndex), verts.at(other)) > overlapCheckDistance)
                continue;

            if (removed.find(i) == removed.end())
                removed.insert({i, *new vector<int>});

            removed.at(i).push_back(other);

            for (int &indice: indices)
                if (indice == other)
                    indice = currentVertIndex;
        }
    }

    vector<int> &toRemove = *new vector<int>;
    for (auto &pair: removed) {
        insertRange(toRemove, removed.at(pair.first));
    }

    sort(toRemove.begin(), toRemove.end(), greater<int>());

    for (int index: toRemove) {
        Vector2Int l = *new Vector2Int((int) floor(verts.at(index).x / groupSize),
                                       (int) floor(verts.at(index).z / groupSize));

        vector<int> &by = vertsByPos.at(l);
        for (int j = (int) by.size() - 1; j >= 0; --j) {
            if (by.at(j) == index)
                by.erase(by.begin() + j);
        }

        verts.erase(verts.begin() + index);

        for (int &indice: indices)
            if (indice >= index)
                indice = indice - 1;
    }

    for (int i = (int) indices.size() - 1; i >= 0; i--) {
        if (i % 3 != 0)
            continue;

        if (indices.at(i) == indices.at(i + 1) || indices.at(i) == indices.at(i + 2) ||
            indices.at(i + 1) == indices.at(i + 2) ||
            indices.at(i) >= verts.size() || indices.at(i + 1) >= verts.size() ||
            indices.at(i + 2) >= verts.size()) {

            indices.erase(indices.begin() + i);
            indices.erase(indices.begin() + i);
            indices.erase(indices.begin() + i);
        }
    }
}

void FillHoles(vector<Vector3> &verts, vector<int> &indices) {
    map<int, vector<int>> &connectionsByIndex = *new map<int, vector<int>>();
    map<int, vector<int>> &indicesByIndex = *new map<int, vector<int>>();
    for (int i = 0; i < verts.size(); i++) {
        connectionsByIndex.insert({i, *new vector<int>()});
        indicesByIndex.insert({i, *new vector<int>()});
    }

    for (int i = 0; i < indices.size(); i += 3) {
        if (!contains(connectionsByIndex.at(indices.at(i)), indices.at(i + 1)))
            connectionsByIndex.at(indices.at(i)).push_back(indices.at(i + 1));
        if (!contains(connectionsByIndex.at(indices.at(i)), indices.at(i + 2)))
            connectionsByIndex.at(indices.at(i)).push_back(indices.at(i + 2));

        if (!contains(connectionsByIndex.at(indices.at(i + 1)), indices.at(i)))
            connectionsByIndex.at(indices.at(i + 1)).push_back(indices.at(i));
        if (!contains(connectionsByIndex.at(indices.at(i + 1)), indices.at(i + 2)))
            connectionsByIndex.at(indices.at(i + 1)).push_back(indices.at(i + 2));

        if (!contains(connectionsByIndex.at(indices.at(i + 2)), indices.at(i + 1)))
            connectionsByIndex.at(indices.at(i + 2)).push_back(indices.at(i + 1));
        if (!contains(connectionsByIndex.at(indices.at(i + 2)), indices.at(i)))
            connectionsByIndex.at(indices.at(i + 2)).push_back(indices.at(i));

        vector<int> arr = *new vector<int>{indices.at(i), indices.at(i + 1), indices.at(i + 2)};

        insertRange(indicesByIndex.at(indices.at(i)), arr);
        insertRange(indicesByIndex.at(indices.at(i + 1)), arr);
        insertRange(indicesByIndex.at(indices.at(i + 2)), arr);
    }

    for (int i = 0; i < verts.size(); i++) {
        Vector2 p = MathC::XZ(verts.at(i));

        for (int j = 0; j < indices.size(); j += 3) {
            if (indices.at(j) == i || indices.at(j + 1) == i || indices.at(j + 2) == i)
                continue;

            Vector2 a = MathC::XZ(verts.at(indices.at(j))),
                    b = MathC::XZ(verts.at(indices.at(j + 1))),
                    c = MathC::XZ(verts.at(indices.at(j)));

            if (!MathC::PointWithinTriangle2DWithTolerance(p, a, b, c))
                continue;

            Vector2 &close1 = MathC::ClosetPointOnLine(p, a, b),
                    &close2 = MathC::ClosetPointOnLine(p, a, c),
                    &close3 = MathC::ClosetPointOnLine(p, b, b);

            Vector2 &close = close3;
            if (Vector2::Distance(close1, p) < Vector2::Distance(close2, p) &&
                Vector2::Distance(close1, p) < Vector2::Distance(close3, p))
                close = close1;
            else if (Vector2::Distance(close2, p) < Vector2::Distance(close3, p))
                close = close2;

            Vector2 offset = close - p;
            Vector3 o = MathC::XYZ(offset.Normalize() * (offset.Magnitude() + .01f));
            verts.at(i) = verts.at(i) + o;
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

                if (final <= original || !contains(connectionsByIndex.at(final), other))
                    continue;

                bool denied = false;

                Vector2 a = MathC::XZ(verts.at(original)),
                        b = MathC::XZ(verts.at(other)),
                        c = MathC::XZ(verts.at(final));
                Vector2 center = Vector2::Lerp(Vector2::Lerp(a, b, .5f), c, .5f);

                float minX = MathC::Min(MathC::Min(a.x, b.x), c.x),
                        minY = MathC::Min(MathC::Min(a.y, b.y), c.y),
                        maxX = MathC::Max(MathC::Max(a.x, b.x), c.x),
                        maxY = MathC::Max(MathC::Max(a.y, b.y), c.y);

                for (int x = 0; x < indices.size(); x += 3) {
                    vector<int> checkArr = *new vector<int>{indices.at(x), indices.at(x + 1), indices.at(x + 2)};
                    if (contains(checkArr, original) &&
                        contains(checkArr, other) &&
                        contains(checkArr, final)) {
                        //The triangle already exists
                        denied = true;
                        break;
                    }

                    Vector2 aP = MathC::XZ(verts.at(checkArr.at(0))),
                            bP = MathC::XZ(verts.at(checkArr.at(1))),
                            cP = MathC::XZ(verts.at(checkArr.at(2)));

                    //Bounding
                    if (maxX < MathC::Min(MathC::Min(aP.x, bP.x), cP.x) ||
                        maxY < MathC::Min(MathC::Min(aP.y, bP.y), cP.y) ||
                        minX > MathC::Max(MathC::Max(aP.x, bP.x), cP.x) ||
                        minY > MathC::Max(MathC::Max(aP.y, bP.y), cP.y))
                        continue;

                    //One of the new triangle points is within an already existing triangle
                    if (MathC::PointWithinTriangle2DWithTolerance(center, aP, bP, cP) ||
                        MathC::PointWithinTriangle2DWithTolerance(a, aP, bP, cP) ||
                        MathC::PointWithinTriangle2DWithTolerance(b, aP, bP, cP) ||
                        MathC::PointWithinTriangle2DWithTolerance(c, aP, bP, cP)) {
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

                indices.push_back(original);
                indices.push_back(other);
                indices.push_back(final);
            }
        }
    }
}

void SetupNavTriangles(vector<int> &indices, vector<NavMeshTriangle> &triangles,
                       map<int, vector<int>> &trianglesByVertexID) {
    for (int i = 0; i < indices.size(); i += 3) {
        int a = indices.at(i), b = indices.at(i + 1), c = indices.at(i + 2);
        NavMeshTriangle triangle = *new NavMeshTriangle(i / 3, a, b, c);

        triangles.push_back(triangle);

        int tID = (int) triangles.size() - 1;

        if (trianglesByVertexID.find(a) == trianglesByVertexID.end())
            trianglesByVertexID.insert({a, *new vector<int>});
        if (trianglesByVertexID.find(b) == trianglesByVertexID.end())
            trianglesByVertexID.insert({b, *new vector<int>});
        if (trianglesByVertexID.find(c) == trianglesByVertexID.end())
            trianglesByVertexID.insert({c, *new vector<int>});

        trianglesByVertexID.at(a).push_back(tID);
        trianglesByVertexID.at(b).push_back(tID);
        trianglesByVertexID.at(c).push_back(tID);
    }
}

void SetupNeighbors(vector<NavMeshTriangle> &triangles,
                    map<int, vector<int>> &trianglesByVertexID) {
    for (int i = 0; i < triangles.size(); i++) {
        vector<int> &neighbors = *new vector<int>();
        vector<int> &possibleNeighbors = *new vector<int>();

        insertRange(possibleNeighbors, trianglesByVertexID.at(triangles.at(i).vertices().at(0)));
        insertRange(possibleNeighbors, trianglesByVertexID.at(triangles.at(i).vertices().at(1)));
        insertRange(possibleNeighbors, trianglesByVertexID.at(triangles.at(i).vertices().at(2)));

        for (int &t: possibleNeighbors) {
            if (t == i)
                continue;

            if (contains(neighbors, t))
                continue;

            if (SharedBetween(triangles.at(i).vertices(), triangles.at(t).vertices()).size() == 2)
                neighbors.push_back(t);
        }

        triangles.at(i).SetNeighborIds(neighbors);
    }
}

vector<int> &SharedBetween(vector<int> &v1, vector<int> &v2) {
    vector<int> &result = *new vector<int>;
    for (const auto &item1: v1) {
        for (const auto &item2: v2) {
            if (item1 == item2)
                result.push_back(item1);
        }
    }

    return result;
}

bool contains(vector<int> &v, int &target) {
    for (const int &item: v) // NOLINT(*-use-anyofallof)
        if (item == target)
            return true;

    return false;
}

bool contains(vector<Vector3> &v, Vector3 &target) {
    for (const Vector3 &item: v) // NOLINT(*-use-anyofallof)
        if (item == target)
            return true;

    return false;
}

int indexOf(vector<Vector3> &list, Vector3 &element) {
    for (int i = 0; i < list.size(); ++i) {
        if (list.at(i) == element)
            return i;
    }

    return -1;
}

bool hasKey(map<Vector2Int, vector<int>> &mapTarget, Vector2Int &key) {
    for (const auto &item: mapTarget) {
        if (item.first == key)
            return true;
    }

    return false;
}

bool hasKey(map<int, vector<int>> &mapTarget, int &key) {
    for (const auto &item: mapTarget) {
        if (item.first == key)
            return true;
    }

    return false;
}

void insertRange(vector<int> &target, vector<int> &from) {
    for (const auto &item: from)
        target.push_back(item);
}

int main() {
    const int averageCount = 1;

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

                cout << "Start vertex count: " << navMeshImport->getVertices()->size() << "\n";
                cout << "Start indices count: " << navMeshImport->getIndices()->size() << "\n\n";

                Vector3 &cleanPoint = *new Vector3(navMeshImport->getCleanPoint()->at(0),
                                                   navMeshImport->getCleanPoint()->at(1),
                                                   navMeshImport->getCleanPoint()->at(2));

                vector<Vector3> &vertices = *new vector<Vector3>;
                for (const vector<float> i: *navMeshImport->getVertices())
                    vertices.push_back(*new Vector3(i.at(0), i.at(1), i.at(2)));

                vector<int> &indices = *new vector<int>;
                for (const int i: *navMeshImport->getIndices())
                    indices.push_back(i);

                auto timerStart = high_resolution_clock::now();

                NavMeshOptimized &navMeshOptimized = OptimizeNavMesh(cleanPoint, vertices, indices);

                auto timerEnd = high_resolution_clock::now();

                auto time = duration_cast<milliseconds>(timerEnd - timerStart);

                total_time += time.count();

                cout << "Vertex count match: "
                     << (((int) navMeshOptimized.getVertices().size()) == navMeshImport->FV())
                     << " | " << navMeshImport->FV() - ((int) navMeshOptimized.getVertices().size()) << "\n";
                cout << "Vertex count match: "
                     << (((int) navMeshOptimized.getIndices().size()) == navMeshImport->FI())
                     << " | " << navMeshImport->FI() - ((int) navMeshOptimized.getIndices().size()) << "\n";
                cout << "Vertex count match: "
                     << (((int) navMeshOptimized.getTriangles().size()) == navMeshImport->FT())
                     << " | " << navMeshImport->FT() - ((int) navMeshOptimized.getTriangles().size()) << "\n\n";

                cout << "Final vertex count: " << navMeshOptimized.getVertices().size() << "\n";
                cout << "Final indices count: " << navMeshOptimized.getIndices().size() << "\n";
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