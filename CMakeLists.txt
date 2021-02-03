cmake_minimum_required(VERSION 3.1)

project(URIFramework CXX)
enable_testing()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1z -lstdc++fs -Wall")
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake;${CMAKE_MODULE_PATH}")

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Eigen3 REQUIRED Core)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(Boost 1.54.0 COMPONENTS serialization REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

include_directories(utils/fast-cpp-csv-parser)
include_directories(utils/CSVWriter/include)

add_subdirectory(utils/ur_rtde)
add_subdirectory(utils/spdlog)
add_subdirectory(src/tests)
