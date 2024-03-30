#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>
#include <map>

using json = nlohmann::json;

using namespace std;
namespace fs = filesystem;

#include "NavMeshImport.h"
#include "NavMeshOptimized.h"
#include "MathC.h"


void CheckOverlap(vector<vector<double>> &verts, vector<int> &indices,
                  map<vector<double>, vector<int>> &vertsByPosition, double size);

bool contains(vector<int> &vector, int &target);

bool contains(vector<vector<double>> &v, vector<double> &target);

vector<double> offsetVector(vector<double> v, int x, int y);

void insertIntoVector(vector<int> &target, vector<int> &from);

void SetupNavTriangles(vector<vector<double>> verts, vector<int> indices, vector<NavMeshTriangle> &triangles,
                       map<int, vector<int>> &trianglesByVertexId);

void SetupNeighbors(vector<NavMeshTriangle> &triangles, map<int, vector<int>> &trianglesByVertexId);

int indexOf(vector<vector<double>> &list, vector<double> &element);

void FillHoles(vector<vector<double>> &verts, vector<int> &indices);

vector<double> XZ(vector<double> &vector);

vector<double> &XYZ(vector<double> &v);

NavMeshImport *load_json_to_nav_mesh_import(fs::path &file) {
    cout << "   Importing navigation mesh from file:" << "\n";
    cout << "   " << file << "\n";

    ifstream str(file);
    json js = json::parse(str);


    vector<double> cleanPoint = *new vector<double>{js["cleanPoint"]["x"],
                                                    js["cleanPoint"]["y"],
                                                    js["cleanPoint"]["z"]};
    cout << "   Clean Point {" << cleanPoint[0] << " , " << cleanPoint[1] << " , " << cleanPoint[2] << "}" << "\n";

    vector<vector<double>> vertexPoints = *new vector<vector<double>>;

    for (int i = 0; i < js["x"].size(); ++i) {
        vertexPoints.push_back(vector<double>{js["x"][i], js["y"][i], js["z"][i]});
    }

    cout << "   Vertex count: " << vertexPoints.size() << "\n";

    vector<int> indices = *new vector<int>;

    for (const auto &item: js["indices"])
        indices.push_back(item);

    cout << "   Indices count: " << indices.size() << "\n\n";

    return new NavMeshImport(cleanPoint, vertexPoints, indices);
}

NavMeshOptimized &optimize_nav_mesh(NavMeshImport import) {
#pragma region Check Vertices and Indices for overlap
    vector<vector<double>> &verts = *import.get_vertices();
    vector<int> &indices = *import.get_indices();

    map<vector<double>, vector<int>> vertsByPosition = *new map<vector<double>, vector<int>>;

    cout << "Start vertex count: " << verts.size() << "\n";
    cout << "Start indices count: " << indices.size() << "\n";

    const double groupSize = 5;

    for (int i = 0; i < verts.size(); ++i) {
        vector<double> v = verts[i];
        vector<double> id = {floor(v[0] / groupSize), floor(v[2] / groupSize)};

        if (vertsByPosition.find(id) == vertsByPosition.end())
            vertsByPosition.insert({id, *new vector<int>});

        vertsByPosition[id].push_back(i);
    }

    CheckOverlap(verts, indices, vertsByPosition, groupSize);

    cout << "Vertex count after overlap check: " << verts.size() << "\n";
    cout << "Indices count after overlap check: " << indices.size() << "\n";

#pragma endregion

#pragma region Create first iteration of NavTriangles

    vector<NavMeshTriangle> triangles = *new vector<NavMeshTriangle>;
    map<int, vector<int>> trianglesByVertexId = *new map<int, vector<int>>;

    SetupNavTriangles(verts, indices, triangles, trianglesByVertexId);

    SetupNeighbors(triangles, trianglesByVertexId);

    cout << "NavTriangles created: " << triangles.size() << "\n";

#pragma endregion

#pragma region Check neighbor connections
    vector<double> cleanPoint = *import.get_clean_point();

    int closestVertex = 0;
    double closestDistance = MathC::quick_square_distance(cleanPoint, verts[closestVertex]);

    for (int i = 1; i < verts.size(); ++i) {
        double d = MathC::quick_square_distance(cleanPoint, verts[i]);

        if (d >= closestDistance)
            continue;

        closestDistance = d;
        closestVertex = i;
    }

    vector<int> connected = *new vector<int>;
    vector<int> toCheck = *new vector<int>;
    insertIntoVector(toCheck, trianglesByVertexId[closestVertex]);

    while (!toCheck.empty()) {
        int index = toCheck[0];
        NavMeshTriangle navMeshTriangle = triangles[index];
        toCheck.erase(toCheck.begin());
        connected.push_back(index);
        for (int &item: navMeshTriangle.neighbors())
            if (!contains(toCheck, item) && !contains(connected, item))
                toCheck.push_back(item);
    }

    cout << "Connected NavTriangle count: " << connected.size() << "\n";

#pragma endregion

#pragma region Fill holes and final iteration of NavTriangles

    vector<vector<double>> fixedVertices = *new vector<vector<double>>;
    vector<int> fixedIndices = *new vector<int>;
    map<int, vector<int>> fixedTrianglesByVertexId = *new map<int, vector<int>>;

    for (int &index: connected) {
        for (const int &vertex: triangles[index].vertices()) {
            if (!contains(fixedVertices, verts[vertex]))
                fixedVertices.push_back(verts[vertex]);

            fixedIndices.push_back(indexOf(fixedVertices, verts[vertex]));

            vector<double> id = *new vector<double>{floor(verts[vertex][0] / groupSize),
                                                    floor(verts[vertex][2] / groupSize)};
            if (vertsByPosition.find(id) == vertsByPosition.end()) {
                int i = indexOf(fixedVertices, verts[vertex]);
                if (!contains(vertsByPosition[id], i))
                    vertsByPosition.insert({id, *new vector<int>{i}});
            } else
                vertsByPosition.insert({id, *new vector<int>{indexOf(fixedVertices, verts[vertex])}});
        }
    }

    FillHoles(fixedVertices, fixedIndices);

    vector<NavMeshTriangle> fixedTriangles = *new vector<NavMeshTriangle>;

    SetupNavTriangles(fixedVertices, fixedIndices, fixedTriangles, fixedTrianglesByVertexId);

    SetupNeighbors(fixedTriangles, fixedTrianglesByVertexId);

    for (NavMeshTriangle &t: fixedTriangles)
        t.set_border_width(fixedVertices, fixedTriangles);

#pragma endregion

    NavMeshOptimized &result = *new NavMeshOptimized();
    result.set_values(fixedVertices, fixedTriangles, groupSize);

    return result;
}

void FillHoles(vector<vector<double>> &verts, vector<int> &indices) {
    map<int, vector<int>> connectionsByIndex = *new map<int, vector<int>>;
    map<int, vector<int>> indicesByIndex = *new map<int, vector<int>>;

    for (int i = 0; i < verts.size(); ++i) {
        connectionsByIndex.insert({i, *new vector<int>});
        indicesByIndex.insert({i, *new vector<int>});
    }

    for (int i = 0; i < indices.size(); i += 3) {
        if (!contains(connectionsByIndex[indices[i]], indices[i + 1]))
            connectionsByIndex[i].push_back(indices[i + 1]);
        if (!contains(connectionsByIndex[indices[i]], indices[i + 2]))
            connectionsByIndex[i].push_back(indices[i + 2]);

        if (!contains(connectionsByIndex[indices[i + 1]], indices[i]))
            connectionsByIndex[i + 1].push_back(indices[i]);
        if (!contains(connectionsByIndex[indices[i + 1]], indices[i + 2]))
            connectionsByIndex[i + 1].push_back(indices[i + 2]);

        if (!contains(connectionsByIndex[indices[i + 2]], indices[i + 1]))
            connectionsByIndex[i + 2].push_back(indices[i + 1]);
        if (!contains(connectionsByIndex[indices[i + 2]], indices[i]))
            connectionsByIndex[i + 2].push_back(indices[i]);

        for (int j = 0; j < 3; ++j) {
            indicesByIndex[indices[i + j]].push_back(indices[i]);
            indicesByIndex[indices[i + j]].push_back(indices[i + 1]);
            indicesByIndex[indices[i + j]].push_back(indices[i + 2]);
        }
    }

    for (int i = 0; i < verts.size(); ++i) {
        vector<double> p = XZ(verts[i]);

        for (int j = 0; j < indices.size(); j += 3) {
            if (indices[j] == i || indices[j + 1] == i || indices[j + 2] == i)
                continue;

            vector<double> a = XZ(verts[indices[j]]),
                    b = XZ(verts[indices[j + 1]]),
                    c = XZ(verts[indices[j + 2]]);

            if (!MathC::points_within_triangle_2d(p, a, b, c))
                continue;

            vector<double> &closeAB = MathC::closet_point_on_line(p, a, b),
                    &closeAC = MathC::closet_point_on_line(p, a, c),
                    &closeCB = MathC::closet_point_on_line(p, c, b);

            vector<double> close;

            if (MathC::distance(closeAB, p) < MathC::distance(closeAC, p) &&
                MathC::distance(closeAB, p) < MathC::distance(closeCB, p))
                close = closeAB;
            else if (MathC::distance(closeAC, p) < MathC::distance(closeCB, p))
                close = closeAC;
            else
                close = closeCB;

            vector<double> offset = MathC::minus(close, p);

            float distance = MathC::distance(offset) + .01f;
            vector<double> norm = MathC::normalize(offset);
            MathC::addVectors(verts[i], MathC::multiply(norm, distance));
        }
    }

    for (int original = 0; original < verts.size(); ++original) {
        vector<int> originalConnections = connectionsByIndex[original];

        for (int otherIndex = 0; otherIndex < originalConnections.size(); ++otherIndex) {
            int other = originalConnections[otherIndex];

            if (other <= original)
                continue;

            for (int finalIndex = otherIndex + 1; finalIndex < originalConnections.size(); ++finalIndex) {
                int final = originalConnections[finalIndex];

                if (final <= original || !contains(connectionsByIndex[final], other))
                    continue;

                bool denied = false;

                vector<double> a = XZ(verts[original]), b = XZ(verts[other]), c = XZ(verts[final]);
                vector<double> center = MathC::lerpVectors(MathC::lerpVectors(a, b, .5), c, .5);

                double minX = MathC::Min(MathC::Min(a[0], b[0]), c[0]),
                        minY = MathC::Min(MathC::Min(a[1], b[1]), c[1]),
                        maxX = MathC::Max(MathC::Max(a[0], b[0]), c[0]),
                        maxY = MathC::Max(MathC::Max(a[1], b[1]), c[1]);

                for (int x = 0; x < indices.size(); x += 3) {
                    vector<int> checkArr = *new vector<int>{indices[x], indices[x + 1], indices[x + 2]};

                    if (contains(checkArr, original) && contains(checkArr, other) && contains(checkArr, final)) {
                        denied = true;
                        break;
                    }

                    vector<double> aP = XZ(verts[checkArr[0]]),
                            bP = XZ(verts[checkArr[1]]),
                            cP = XZ(verts[checkArr[2]]);

                    if (maxX < MathC::Min(MathC::Min(aP[0], bP[0]), cP[0]) ||
                        maxY < MathC::Min(MathC::Min(aP[1], bP[1]), cP[1]) ||
                        minX > MathC::Max(MathC::Max(aP[0], bP[0]), cP[0]) ||
                        minY > MathC::Max(MathC::Max(aP[1], bP[1]), cP[1]))
                        continue;

                    if (MathC::points_within_triangle_2d(center, aP, bP, cP) ||
                        MathC::points_within_triangle_2d(a, aP, bP, cP) ||
                        MathC::points_within_triangle_2d(b, aP, bP, cP) ||
                        MathC::points_within_triangle_2d(c, aP, bP, cP)) {
                        denied = true;
                        break;
                    }

                    if (!MathC::triangle_intersect_2d(a, b, c, aP, bP, cP))
                        continue;

                    denied = true;
                    break;
                }

                if (denied)
                    continue;

                cout << "Fill" << "\n";

                indices.push_back(original);
                indices.push_back(other);
                indices.push_back(final);
            }
        }
    }
}

vector<double> &XYZ(vector<double> &v) {
    return *new vector<double>{v[0], 0, v[1]};
}

vector<double> XZ(vector<double> &vector) {
    return {vector[0], vector[2]};
}

int indexOf(vector<vector<double>> &list, vector<double> &element) {
    for (int i = 0; i < list.size(); ++i) {
        if (list[i] == element)
            return i;
    }

    return -1;
}

void SetupNeighbors(vector<NavMeshTriangle> &triangles, map<int, vector<int>> &trianglesByVertexId) {
    for (int i = 0; i < triangles.size(); ++i) {
        vector<int> neighbors = *new vector<int>;
        vector<int> possibleNeighbors = *new vector<int>;

        insertIntoVector(possibleNeighbors, trianglesByVertexId[triangles[i].GetA()]);
        insertIntoVector(possibleNeighbors, trianglesByVertexId[triangles[i].GetB()]);
        insertIntoVector(possibleNeighbors, trianglesByVertexId[triangles[i].GetC()]);

        for (const auto &t: possibleNeighbors) {
            if (t == i)
                continue;

            if (MathC::t_shared_between(triangles[i].vertices(), triangles[t].vertices(), 2).size() == 2)
                neighbors.push_back(t);

            if (neighbors.size() == 3)
                break;
        }

        triangles[i].SetNeighborIds(neighbors);
    }
}

void SetupNavTriangles(vector<vector<double>> verts, vector<int> indices, vector<NavMeshTriangle> &triangles,
                       map<int, vector<int>> &trianglesByVertexId) {
    for (int i = 0; i < indices.size(); i += 3) {
        int a = indices[i], b = indices[i + 1], c = indices[i + 2];
        NavMeshTriangle triangle = *new NavMeshTriangle(i / 3, a, b, c, verts);

        triangles.push_back(triangle);

        int tId = triangles.size() - 1;

        if (trianglesByVertexId.find(a) == trianglesByVertexId.end())
            trianglesByVertexId.insert({a, *new vector<int>});
        if (trianglesByVertexId.find(b) == trianglesByVertexId.end())
            trianglesByVertexId.insert({b, *new vector<int>});
        if (trianglesByVertexId.find(c) == trianglesByVertexId.end())
            trianglesByVertexId.insert({c, *new vector<int>});

        trianglesByVertexId[a].push_back(tId);
        trianglesByVertexId[b].push_back(tId);
        trianglesByVertexId[c].push_back(tId);
    }
}

void CheckOverlap(vector<vector<double>> &verts, vector<int> &indices,
                  map<vector<double>, vector<int>> &vertsByPosition, const double groupSize) {
    const double overlapCheckDistance = .3;
    const double divided = 20;
    map<int, vector<int>> removed = *new map<int, vector<int>>;

    for (int original = 0; original < verts.size(); ++original) {
        if (removed.find(floor(original / divided)) != removed.end())
            if (contains(removed[floor(original / divided)], original))
                continue;

        vector<double> id = {floor(verts[original][0] / groupSize), floor(verts[original][2] / groupSize)};
        vector<int> toCheck = *new vector<int>;

        if (vertsByPosition.find(id) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[id]);

        if (vertsByPosition.find(offsetVector(id, -1, -1)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, -1, -1)]);

        if (vertsByPosition.find(offsetVector(id, -1, 0)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, -1, 0)]);

        if (vertsByPosition.find(offsetVector(id, -1, 1)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, -1, 1)]);

        if (vertsByPosition.find(offsetVector(id, 0, -1)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, 0, -1)]);

        if (vertsByPosition.find(offsetVector(id, 0, 1)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, 0, 1)]);

        if (vertsByPosition.find(offsetVector(id, 1, -1)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, 1, -1)]);

        if (vertsByPosition.find(offsetVector(id, 1, 0)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, 1, 0)]);

        if (vertsByPosition.find(offsetVector(id, 1, 1)) != vertsByPosition.end())
            insertIntoVector(toCheck, vertsByPosition[offsetVector(id, 1, 1)]);

        for (int &other: toCheck) {
            if (other == original)
                continue;

            if (removed.find(floor(other / divided)) != removed.end())
                if (contains(removed[floor(other / divided)], other))
                    continue;

            if (MathC::distance(verts[original], verts[other]) > overlapCheckDistance)
                continue;

            if (removed.find(floor(other / divided)) == removed.end())
                removed.insert({floor(other / divided), *new vector<int>});

            removed[floor(other / divided)].push_back(other);

            for (int &i: indices) {
                if (i == other)
                    i = original;
            }
        }
    }

    vector<int> toRemove = *new vector<int>;
    for (auto &item: removed) {
        int i = item.first;
        if (!contains(toRemove, i))
            toRemove.push_back(item.first);
    }

    sort(toRemove.begin(), toRemove.end(), greater<int>()); // NOLINT(*-use-transparent-functors)

    for (const auto &index: toRemove) {
        vector<int> &v = vertsByPosition[{floor(verts[index][0] / groupSize), floor(verts[index][2] / groupSize)}];
        remove(v.begin(), v.end(), index); // NOLINT(*-unused-return-value)
        verts.erase(verts.begin() + index);
    }

    for (int i = indices.size() - 1; i >= 0; --i) {
        if (i % 3 != 0)
            continue;

        if (indices[i] != indices[i + 1] && indices[i] != indices[i + 2] && indices[i + 1] != indices[i + 2] &&
            indices[i] < verts.size() && indices[i + 1] < verts.size() && indices[i + 2] < verts.size())
            continue;

        indices.erase(indices.begin() + i);
        indices.erase(indices.begin() + i);
        indices.erase(indices.begin() + i);
    }
}

void insertIntoVector(vector<int> &target, vector<int> &from) {
    target.insert(end(target), begin(from), end(from));
}


vector<double> offsetVector(vector<double> v, int x, int y) {
    return *new vector<double>{v[0] + x, v[1] + y};
}

bool contains(vector<int> &vector, int &target) {
    for (int &i: vector) { // NOLINT(*-use-anyofallof)
        if (i == target)
            return true;
    }

    return false;
}

bool contains(vector<vector<double>> &v, vector<double> &target) {
    for (vector<double> &i: v) { // NOLINT(*-use-anyofallof)
        if (i == target)
            return true;
    }

    return false;
}

int main() {
    const vector<string> file_letter = {"S", "M", "L"};
    const fs::path folder_path = fs::current_path().parent_path().parent_path() += "\\JsonFiles\\";
    cout << "Using json text files from folder:\n"<<folder_path << "\n";

    for (int letter_index = 0; letter_index < 3; ++letter_index) {
        for (int number_index = 1; number_index <= 5; ++number_index) {
            constexpr int average_count = 10;

            cout << "Optimization for: " << file_letter.at(letter_index) << " " << number_index << '\n';

            fs::path fileName = fs::current_path().parent_path().parent_path() +=
                                        "\\JsonFiles\\" +
                                        file_letter.at(letter_index) +
                                        " " + to_string(number_index) +
                                        ".txt";

            NavMeshImport *nav_mesh_import = load_json_to_nav_mesh_import(fileName);

            long long total_time = 0.0;

            for (int i = 0; i < average_count; ++i) {

                cout << "Check " << to_string(i + 1) << "\n";
                auto start = chrono::high_resolution_clock::now();

                NavMeshOptimized &nav_mesh_optimized = optimize_nav_mesh(*nav_mesh_import);

                auto end = chrono::high_resolution_clock::now();

                auto microseconds = chrono::duration_cast<std::chrono::microseconds>(end - start);

                total_time += microseconds.count();

                cout << "Final vertex count: " << nav_mesh_optimized.get_vertices().size() << "\n";
                cout << "Final triangle count: " << nav_mesh_optimized.get_triangles()->size() << "\n";
                cout << "Time: " << microseconds.count() << "(ms)\n";
                cout << "Time: " << microseconds.count() / 1000 << "(s)\n\n";
            }

            cout << "Repeat count: " << average_count << "\n";
            cout << "Total time for repeats: " << total_time << "(ms)\n";
            cout << "Total time for repeats: " << total_time / 1000 << "(s)\n";
            cout << "Average time for " << file_letter.at(letter_index) << " " << number_index << ": "
                 << total_time / average_count << "(ms)\n";
            cout << "Average time for " << file_letter.at(letter_index) << " " << number_index << ": "
                 << total_time / average_count / 1000 << "(s)\n\n";
        }

    }

    return 0;
}