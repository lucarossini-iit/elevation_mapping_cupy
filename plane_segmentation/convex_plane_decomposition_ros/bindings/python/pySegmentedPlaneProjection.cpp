#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <convex_plane_decomposition_msgs/PlanarTerrain.h>
#include <convex_plane_decomposition_msgs/PlanarRegion.h>
#include <convex_plane_decomposition_msgs/BoundingBox2d.h>
#include <convex_plane_decomposition_msgs/PolygonWithHoles2d.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/embed.h>
#include <pybind11/detail/common.h>
#include <pybind11/stl.h>

using namespace convex_plane_decomposition;

namespace py = pybind11;

std::shared_ptr<ros::NodeHandle> nh_ptr;
ros::Subscriber sub, sub_boundaries;
ros::Publisher pub_boundaries;
std::vector<PlanarRegion> planar_regions, planar_regions_temp;
visualization_msgs::MarkerArray boundaries;

void callback(const convex_plane_decomposition_msgs::PlanarTerrain::ConstPtr& msg)
{
    auto planar_terrain = fromMessage(*msg);

    planar_regions_temp = planar_terrain.planarRegions;
}

void boundaries_callback(const visualization_msgs::MarkerArrayConstPtr& msg)
{
    boundaries = *msg;
}

auto project = [](Eigen::Vector3d queryPoint, Eigen::Vector2d desiredVelocity)
{
    ros::spinOnce();

    auto penaltyFunction = [&queryPoint, &desiredVelocity](const Eigen::Vector3d& projectedPoint) { 
         
        Eigen::Vector2d projectedPointXY = projectedPoint.head(2);
        Eigen::Vector2d queryPointXY = queryPoint.head(2);
        Eigen::Vector2d normalizedDelta = (projectedPointXY - queryPointXY).normalized();
        Eigen::Vector2d normalizedVel = desiredVelocity.normalized();
        
        //Compute scalar product (-1 < m < 1) and translate it to (0 < m < 2)
        double metric = normalizedVel.dot(normalizedDelta) + 1; 
        double normVel = desiredVelocity.norm();
        double k = 0.2;

        double normDelta = (projectedPointXY - queryPointXY).norm();
        double ts = 0.01; // threshold for triggering penalty function 
        double barrier = normDelta < ts ? 0 : 1; 
        
        return barrier*k*std::exp(-normVel * (metric*metric));
    };

    std::cout << "Planar regions size: " << planar_regions_temp.size() << std::endl; 
    auto projection = getBestPlanarRegionAtPositionInWorld(queryPoint, planar_regions_temp, penaltyFunction);

    return projection.positionInWorld;
};


auto update = []()
{
    planar_regions = planar_regions_temp;
    pub_boundaries.publish(boundaries);
};

//std::function<void()> update;
//void updateHandler() { update(); };

// bool init (std::string name, std::list<std::string> args)
// {
//     if(ros::ok())
//     {
//         ROS_ERROR("Ros node already initialized with name %s",
//                   ros::this_node::getName().c_str());
//         // return false;
//     }

//     std::vector<const char *> args_vec;
//     for(auto& a : args)
//     {
//         args_vec.push_back(a.c_str());
//     }

//     int argc = args_vec.size();

//     char ** argv = (char **)args_vec.data();

//     name += "_cpp";

//     ros::init(argc, argv, name, ros::init_options::NoSigintHandler);

//     ros::NodeHandle nh;

//     ROS_INFO("Initialized roscpp under namespace %s with name %s",
//              ros::this_node::getNamespace().c_str(),
//              ros::this_node::getName().c_str()
//             );
    
// //    update = []()
// //    {
// //    //    ros::spinOnce();
// //        std::cout << "update" << std::endl;
// //        auto planar_terrain_ptr = ros::topic::waitForMessage<convex_plane_decomposition_msgs::PlanarTerrain>("/convex_plane_decomposition_ros/planar_terrain", *nh, ros::Duration(0.5));
// //        std::cout << "planar terrain message found" << std::endl;
// //        auto planar_terrain = fromMessage(*planar_terrain_ptr);
// //        planar_regions = planar_terrain.planarRegions;


// //        auto boundaries_ptr = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("/convex_plane_decomposition_ros/boundaries", *nh, ros::Duration(0.5));
// //        std::cout << "boundaries message found" << std::endl;
// //        boundaries = *boundaries_ptr;
// //        pub_boundaries.publish(boundaries);
// //    };


//     sub = nh.subscribe("/convex_plane_decomposition_ros/planar_terrain", 1, callback);
//     sub_boundaries = nh.subscribe("/convex_plane_decomposition_ros/boundaries", 1, boundaries_callback);
//     pub_boundaries = nh.advertise<visualization_msgs::MarkerArray>("/convex_plane_decomposition_ros/boundaries/update", 1, true);

//     return true;
// }

bool init(const std::string& name, const std::list<std::string>& args)
{
    if (!ros::isInitialized())
    {
        // Convert std::list<std::string> to argc/argv format
        std::vector<const char*> args_vec;
        for (const auto& a : args)
        {
            args_vec.push_back(a.c_str());
        }

        int argc = args_vec.size();
        char** argv = const_cast<char**>(args_vec.data());

        // Initialize ROS
        ros::init(argc, argv, name + "_cpp", ros::init_options::NoSigintHandler);
        ROS_INFO("ROS initialized with name: %s", (name + "_cpp").c_str());
    }
    else
    {
        ROS_WARN("ROS is already initialized. Using the existing node handle.");
    }

    // Create a new NodeHandle if not already created
    if (!nh_ptr)
    {
        nh_ptr = std::make_shared<ros::NodeHandle>();
        ROS_INFO("NodeHandle created.");
    }

    // Set up subscribers and publishers
    sub = nh_ptr->subscribe("/convex_plane_decomposition_ros/planar_terrain", 1, callback);
    sub_boundaries = nh_ptr->subscribe("/convex_plane_decomposition_ros/boundaries", 1, boundaries_callback);
    pub_boundaries = nh_ptr->advertise<visualization_msgs::MarkerArray>("/convex_plane_decomposition_ros/boundaries/update", 1, true);

    ROS_INFO("Subscribers and publishers set up.");
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
    m.def("update", update);
}
