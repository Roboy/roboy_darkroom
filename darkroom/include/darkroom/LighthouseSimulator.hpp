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
#include <sensor_msgs/Imu.h>
#include "darkroom/Transform.hpp"
#include "darkroom/Sensor.hpp"
#include "darkroom/Utilities.hpp"
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include "darkroom/Triangulation.hpp"

#define degreesToTicks(degrees) (degrees * 50.0 * 8333.0 / 180.0)

#define IMU_ACC_NOISE 0.1

using namespace std;
using namespace Eigen;
using namespace chrono;

class LighthouseSimulator:public rviz_visualization, DarkRoom::Transform, Utilities{
public:
    /**
     * Constructor
     * @param id lighthouse id (currently only 0 or 1 is supported)
     * @param configFile path to yaml file
     * @param object_pose (initial object pose wrt world)
     */
    LighthouseSimulator(int id, fs::path configFile);
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
     * Use feedback from interactive markers to update relative object pose
     * @param msg
     */
    void interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg);

    bool checkIfSensorVisible(vector<Vector4d> &vertices, Vector3d &ray);

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

    int objectID;
    string name;
    fs::path meshPath;
    map<int, Sensor> sensors;
    vector<int> calibrated_sensors;
    boost::shared_ptr<boost::thread> sensor_thread, imu_thread;
    atomic<bool> sensor_publishing, imu_publishing;
    mutex mux;
    tf::Transform relative_object_pose;
    int id;
private:
    ros::NodeHandlePtr nh;
    ros::Publisher sensors_pub, imu_pub;
    ros::Subscriber interactive_marker_sub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    map<int, Vector4d> sensor_position;
    map<int, Vector2d> sensor_angle;
    static int class_counter;
    struct{
        vector<::pcl::Vertices> polygons;
        vector<Vector4d> vertices;
    }mesh;
};

typedef boost::shared_ptr<LighthouseSimulator> LighthouseSimulatorPtr;