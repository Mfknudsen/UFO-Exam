#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

#include "NavMeshImport.h"
#include "NavMeshOptimized.h"


NavMeshImport* LoadJsonToNavMeshImport(const std::string file)
{
    return nullptr;
}

NavMeshOptimized* OptimizeNavMesh(const NavMeshImport& import)
{
#pragma region Check Vertices and Indices for overlap

#pragma endregion

#pragma region Create first iteration of NavTriangles

#pragma endregion

#pragma region Check neighbor connections

#pragma endregion

#pragma region Fill holes and final iteration of NavTriangles

#pragma endregion

    return new NavMeshOptimized();
}

int main()
{
    const std::vector<std::string> file_letter = {"S", "M", "L"};
    const std::filesystem::path folder_path = fs::current_path().parent_path().parent_path().parent_path();
    std::cout << folder_path << "\n";

    for (int letter_index = 0; letter_index < 3; ++letter_index)
    {
        for (int number_index = 1; number_index <= 5; ++number_index)
        {
            constexpr int average_count = 10;
            std::cout << "Optimization for: " << file_letter.at(letter_index) << " " << number_index << '\n';

            std::string fileName = file_letter.at(letter_index) + " " + std::to_string(number_index);

            const NavMeshImport* nav_mesh_import = LoadJsonToNavMeshImport(folder_path.string());

            double total_time = 0.0;

            for (int i = 0; i < average_count; ++i)
            {
                std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::high_resolution_clock::now();

                NavMeshOptimized* nav_mesh_optimized = OptimizeNavMesh(*nav_mesh_import);

                std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::high_resolution_clock::now();

                auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                total_time += microseconds.count();

                std::cout << "Time: " << microseconds.count() << "\n";
                std::cout << "Vertex count: " << nav_mesh_optimized->get_vertices().size() << "\n";
                std::cout << "Triangle count: " << nav_mesh_optimized->get_triangles().size() << "\n";
            }
        }
    }

    return 0;    
}
