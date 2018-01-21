#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <common_utilities/rviz_visualization.hpp>
#include <roboy_communication_middleware/DarkRoom.h>
#include <atomic>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <thread>
#include <chrono>
#include <cstdint>
#include <sensor_msgs/Imu.h>
#include "darkroom/Transform.hpp"
#include "darkroom/Triangulation.hpp"
#include "darkroom/Sensor.hpp"
#include "darkroom/Utilities.hpp"
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>

#define degreesToTicks(degrees) (degrees * 50.0 * 8333.0 / 180.0)
#define radiansToTicks(radians) (radians * 50.0 * 8333.0 / M_PI)

#define IMU_ACC_NOISE 0.1

using namespace std;
using namespace Eigen;
using namespace chrono;

class LighthouseSimulator:public rviz_visualization, DarkRoom::Transform, Utilities, public Triangulation{
public:
    /**
     * Constructor
     * @param id lighthouse id (currently only 0 or 1 is supported)
     * @param configFile path to yaml files
     * @param object_pose (initial object pose wrt world)
     */
    LighthouseSimulator(int id, vector<fs::path> &configFile);
    ~LighthouseSimulator();
    /**
     * Publishes simulated lighthouse data
     */
    void PublishSensorData();
    /**
     * Publishes simulated imu data
     */
    void PublishImuData();

    /**
     * Iterates through all polygons and checks for the number of intersections
     * @param vertices the vertices of the mesh
     * @param polygons indices of vertices (only triangles supported)
     * @param ray the ray to check intersections for
     * @return is visible
     */
    bool checkIfSensorVisible(vector<Vector4d> &vertices, vector<::pcl::Vertices> &polygons, Vector3d &ray);

    /**
     * Checks if a ray intersects a triangle
     * (adapted from http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/)
     * @param ray the ray
     * @param v0 first vertex
     * @param v1 second vertex
     * @param v2 third vertex
     * @param u barycentric coordinate
     * @param v barycentric coordinate
     * @param t length of intersection on ray: intersection = t*ray
     * @return intersects
     */
    bool rayIntersectsTriangle(Vector3d &origin, Vector3d &ray,
                               Vector4d &v0, Vector4d &v1, Vector4d &v2,
                               double &u, double &v, double &t);
    boost::shared_ptr<boost::thread> sensor_thread = nullptr, imu_thread = nullptr;
    atomic<bool> sensor_publishing, imu_publishing;
    mutex mux;
    int id;
private:
    ros::NodeHandlePtr nh;
    ros::Publisher sensors_pub;
    ros::Subscriber aruco_pose_sub;
    vector<ros::Publisher> imu_pub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;

    vector<string> objectID;
    vector<string> name;
    vector<string> imu_topic_name;
    bool has_mesh = false;
    struct mesh{
        vector<::pcl::Vertices> polygons;
        vector<Vector4d> vertices;
        vector<Vector4d> vertices_transformed;
    };
    vector<mesh> meshes;
    vector<map<int, Vector4d>> sensor_position;
    vector<vector<bool>> sensor_visible;
};

typedef boost::shared_ptr<LighthouseSimulator> LighthouseSimulatorPtr;
