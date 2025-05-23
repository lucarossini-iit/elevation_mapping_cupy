cmake_minimum_required(VERSION 3.0)

project(elevation_mapping_cupy VERSION 1.0.0)

## add sub-packages
add_subdirectory(elevation_map_msgs)
add_subdirectory(elevation_mapping_cupy)
add_subdirectory(plane_segmentation/grid_map_filters_rsl)
add_subdirectory(plane_segmentation/cgal5_catkin)
add_subdirectory(plane_segmentation/convex_plane_decomposition_msgs)
add_subdirectory(plane_segmentation/convex_plane_decomposition_ros)

