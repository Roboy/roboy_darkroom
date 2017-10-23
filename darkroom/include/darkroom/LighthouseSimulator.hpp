#pragma once

#include <ros/ros.h>
#include <common_utilities/rviz_visualization.hpp>
#include <roboy_communication_middleware/DarkRoom.h>
#include <atomic>
#include <mutex>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "darkroom/Transform.hpp"

#define degreesToTicks(degrees) (degrees * 50.0 * 8333.0 / 180.0)

using namespace std;
using namespace Eigen;

class LighthouseSimulator:public rviz_visualization, DarkRoom::Transform{
public:
    /**
     * Constructor
     * @param id lighthouse id (currently only 0 or 1 is supported)
     * @param object_pose (initial object pose wrt world)
     */
    LighthouseSimulator(int id);
    ~LighthouseSimulator();

    void PublishSensorData();

    boost::shared_ptr<boost::thread> sensor_thread;
    atomic<bool> sensor_publishing;
    mutex mux;
private:
    ros::NodeHandlePtr nh;
    ros::Publisher sensors_pub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    map<int, Vector4d> sensor_position;
    map<int, Vector2d> sensor_angle;
    int id;
    static int class_counter;
};