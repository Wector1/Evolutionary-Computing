cmake_minimum_required(VERSION 3.14)
project(TSPVisualizer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Matplot++ REQUIRED)

add_executable(tsp_visualizer main.cpp)

target_link_libraries(tsp_visualizer PUBLIC Matplot++::matplot)