cmake_minimum_required(VERSION 2.8.11)

project(osrm-batchroute CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Boost 1.58 COMPONENTS filesystem program_options REQUIRED)
find_package(LibOSRM REQUIRED)

add_executable(osrm-batchroute osrm-batchroute.cpp)
target_include_directories(osrm-batchroute PRIVATE ${LibOSRM_INCLUDE_DIRS})
target_link_libraries(osrm-batchroute PRIVATE ${LibOSRM_LIBRARIES} ${LibOSRM_DEPENDENT_LIBRARIES})
target_link_libraries(osrm-batchroute PRIVATE Boost::filesystem Boost::program_options)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")
