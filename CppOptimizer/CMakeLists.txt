cmake_minimum_required(VERSION 3.27)
project(CppOptimizer)

set(CMAKE_CXX_STANDARD 17)

add_executable(CppOptimizer main.cpp
        NavMeshImport.cpp
        NavMeshImport.h
        NavMeshOptimized.cpp
        NavMeshOptimized.h
        NavMeshTriangle.cpp
        NavMeshTriangle.h
        MathC.cpp
        MathC.h)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

target_link_libraries(CppOptimizer PRIVATE nlohmann_json::nlohmann_json)