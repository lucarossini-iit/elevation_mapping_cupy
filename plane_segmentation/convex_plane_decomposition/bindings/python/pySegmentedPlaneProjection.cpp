#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <convex_plane_decomposition_msgs/PlanarRegion.h>
#include <convex_plane_decomposition_msgs/BoundingBox2d.h>
#include <convex_plane_decomposition_msgs/PolygonWithHoles2d.h>

#include <ros/ros.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/detail/common.h>
#include <pybind11/stl.h>

using namespace convex_plane_decomposition;

namespace py = pybind11;

ros::Subscriber sub;
std::vector<PlanarRegion> planar_regions;

void callback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg)
{
    auto planar_terrain = fromMessage(*msg);

    planar_regions = planar_terrain.planarRegions;
    std::cout << "message receied" << std::endl;
}

auto project = [](Eigen::Vector3d query_point)
{
//    auto planar_terrain_msg = ros::topic::waitForMessage<convex_plane_decomposition_msgs::PlanarTerrain>("/convex_plane_decompsosition_ros/planar_terrain", ros::Duration(5.0));

//    std::cout << "here" << std::endl;
//    std::unique_ptr<convex_plane_decomposition::PlanarTerrain> planar_terrain(
//        new convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*planar_terrain_msg)));
//    std::cout << "HERE" << std::endl;

    planar_regions.clear();

    auto start = ros::Time::now();
    while (ros::Time::now() - start < ros::Duration(20) && planar_regions.empty())
    {
        std::cout << ros::Time::now() - start << std::endl;
        ros::spinOnce();
    }

    if (planar_regions.empty())
    {
        std::cout << "no planar region detected." << std::endl;
        return Eigen::Vector3d();
    }

    auto penaltyFunction = [](const Eigen::Vector3d& projectedPoint) { return 0.0; };

    auto projection = getBestPlanarRegionAtPositionInWorld(query_point, planar_regions, penaltyFunction);

    return projection.positionInWorld;
};

bool init (std::string name, std::list<std::string> args)
{
    if(ros::ok())
    {
        ROS_ERROR("Ros node already initialized with name %s",
                  ros::this_node::getName().c_str());
        return false;
    }

    std::vector<const char *> args_vec;
    for(auto& a : args)
    {
        args_vec.push_back(a.c_str());
    }

    int argc = args_vec.size();

    char ** argv = (char **)args_vec.data();

    name += "_cpp";

    ros::init(argc, argv, name, ros::init_options::NoSigintHandler);

    ROS_INFO("Initialized roscpp under namespace %s with name %s",
             ros::this_node::getNamespace().c_str(),
             ros::this_node::getName().c_str()
            );

    ros::NodeHandle nh;
    sub = nh.subscribe("/convex_plane_decomposition_ros/planar_terrain", 1, callback);

    return true;
}

bool shutdown()
{
    if(ros::ok())
    {
        ROS_INFO("Shutting down ros node");
        ros::shutdown();
        return true;
    }

    std::cerr << "Ros node not running" << std::endl;
    return false;
}


PYBIND11_MODULE(pysegmented_plane_projection, m)
{
    py::class_<PlanarRegion>(m, "PlanarRegion")
        .def(py::init<>())
        .def_readwrite("boundaryWithInset", &PlanarRegion::boundaryWithInset)
        .def_readwrite("bbox2d", &PlanarRegion::bbox2d)
        .def_readwrite("transformPlaneToWorld", &PlanarRegion::transformPlaneToWorld);

    m.def("init", init);
    m.def("shutdown", shutdown);
    m.def("project", project);
}
