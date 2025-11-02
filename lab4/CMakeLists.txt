cmake_minimum_required(VERSION 3.10)
project(TSPA_Solver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Matplot++ REQUIRED)

# Default to Release builds if not specified (important for performance)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Optimization flags (safe defaults)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-O3 -DNDEBUG)
endif()

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/problem
    ${PROJECT_SOURCE_DIR}/src/solvers
    ${PROJECT_SOURCE_DIR}/src/utils
)

# Source files
set(SOURCES
    src/main.cpp
    src/problem/TSPAInstance.cpp
    src/problem/Solution.cpp
    src/solvers/RandomSearch.cpp
    src/solvers/NearestNeighbour.cpp
    src/solvers/NearestNeighbourAny.cpp
    src/solvers/NearestNeighbour2Regret.cpp
    src/solvers/NearestNeighbour2RegretWeighted.cpp
    src/solvers/NearestNeighbourCycle.cpp
    src/solvers/NeighbourSteepest.cpp
    src/solvers/kRegretCycle.cpp
    src/solvers/kRegret.cpp
    src/solvers/kRegretCycleWeighted.cpp
    src/solvers/LocalSteepest.cpp
    src/solvers/LocalGreedy.cpp
    src/utils/SolverRunner.cpp
)

# Executable
add_executable(tspa_solver ${SOURCES})

# Link against Matplot++ for visualization
target_link_libraries(tspa_solver PUBLIC Matplot++::matplot)
