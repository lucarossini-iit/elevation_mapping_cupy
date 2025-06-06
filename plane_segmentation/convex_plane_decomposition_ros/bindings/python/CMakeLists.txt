cmake_minimum_required(VERSION 3.15)

find_package(Python3 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11_catkin REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

pybind11_add_module(pysegmented_plane_projection pySegmentedPlaneProjection.cpp)
target_link_libraries(pysegmented_plane_projection
                      PRIVATE
                      convex_plane_decomposition_ros
                      ${convex_plane_decomposition_LIBRARIES}
                      ${catkin_LIBRARIES})

target_include_directories(pysegmented_plane_projection
                           PRIVATE
                           ${pybind11_catkin_INCLUDE_DIRS})

install(TARGETS pysegmented_plane_projection
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
