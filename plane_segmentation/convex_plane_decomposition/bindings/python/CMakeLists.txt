cmake_minimum_required(VERSION 3.15)

find_package(Python3 REQUIRED COMPONENTS Development Interpreter)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(pybind11 REQUIRED)
message(STATUS "**********************************Check for pybind11")
if(${pybind11_FOUND})

    message(STATUS "+++++++++++++++++++++++++++++++Compiling python bindings (${pybind11_LIBRARIES})")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})

    pybind11_add_module(pysegmented_plane_projection pySegmentedPlaneProjection.cpp)
    target_link_libraries(pysegmented_plane_projection PRIVATE convex_plane_decomposition ${convex_plane_decomposition_ros_LIBRARIES} ${catkin_LIBRARIES})
    install(TARGETS pysegmented_plane_projection
        DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})

else()
    message(STATUS "-----------------------------------pybind not found")
endif()

