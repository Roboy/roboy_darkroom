/*
    BSD 3-Clause License

    Copyright (c) 2018, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: simulates a lighthouse
*/

#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <common_utilities/rviz_visualization.hpp>
#include <roboy_middleware_msgs/DarkRoom.h>
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
    void startSensorPublisher();
    void startIMUPublisher();
    bool record(bool start);
private:
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
    map<int,ofstream> file;
    boost::shared_ptr<boost::thread> sensor_thread = nullptr, imu_thread = nullptr;
    atomic<bool> sensor_publishing, imu_publishing, recording;
    mutex mux;
    int id;
};

typedef boost::shared_ptr<LighthouseSimulator> LighthouseSimulatorPtr;
