#include <chrono>
#include <fstream>
#include <fstream>
#include <iostream>
#include <string>

#include "NavMeshImport.h"
#include "NavMeshOptimized.h"


NavMeshImport LoadJsonToNavMeshImport()
{
}

NavMeshOptimized OptimizeNavMesh(const NavMeshImport& import)
{
}

int main()
{
    const std::vector<std::string> file_letter = {"S", "M", "L"};

    for (int letter_index = 0; letter_index < 3; ++letter_index)
    {
        for (int number_index = 1; number_index <= 5; ++number_index)
        {
            constexpr int average_count = 10;
            std::cout << "Optimization for: " << file_letter.at(letter_index) << " " << number_index << '\n';

            const NavMeshImport nav_mesh_import = LoadJsonToNavMeshImport();

            double total_time = 0.0;

            for (int i = 0; i < average_count; ++i)
            {
                std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::high_resolution_clock::now();

                NavMeshOptimized nav_mesh_optimized = OptimizeNavMesh(nav_mesh_import);

                std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::high_resolution_clock::now();

                auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

                total_time += microseconds.count();

                std::cout << "Time: " << microseconds.count() << "\n";
                std::cout << "Vertex count: " << nav_mesh_optimized.GetVertices().size() << "\n";
                std::cout << "Triangle count: " << nav_mesh_optimized.GetTriangles().size() << "\n";
            }
        }
    }

    return 0;
}
