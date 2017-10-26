#pragma once

#include <ros/ros.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <fstream>
#include <deque>
#include <thread>
#include <mutex>
#include <bitset>
#include "yaml-cpp/yaml.h"
#include <common_utilities/rviz_visualization.hpp>
#include <common_utilities/UDPSocket.hpp>
#include "darkroom/LighthouseEstimator.hpp"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

#define uSecsToRadians(ticks) (degreesToRadians(ticks * 0.021600864))
#define ticksToRadians(ticks) (degreesToRadians(ticks * 0.021600864 / 50.0))

using namespace std;

class TrackedObject : public LighthouseEstimator {
public:
    TrackedObject();

    ~TrackedObject();

    /**
     * This function initializes a subscriber for roboy darkroom
     */
    void connectRoboy();
    /**
     * connect an arbitrary object via UDP socket
     * @param broadcastIP your broadcast ip (check with ifconfig)
     * @param port the port to listen on
     */
    void connectObject(const char* broadcastIP, int port);

    /**
     * This function switches the lighthouse ids
     */
    void switchLighthouses(bool switchID);

    /**
     * Triggers recording of sensor data to a sensor.log file
     * @param start
     * @return success (if writing to the file is possible)
     */
    bool record(bool start);

    /**
     * Reads a yaml tracked object file
     * @param filepath to
     * @return success
     */
    bool readConfig(string filepath);

    /**
     * Writes a tracked object to file
     * @param filepath
     * @return success
     */
    bool writeConfig(string filepath);

    static int trackeObjectInstance; //! a unique object instance (helps with unique rviz marker ids)
private:

    /**
     * Continuously receiving, decoding and updating the sensor data from ROS message
     */
    void receiveSensorDataRoboy(const roboy_communication_middleware::DarkRoom::ConstPtr &msg);

    /**
     * Listen for sensor data via UDP
     */
    void receiveSensorData();

public:
    boost::shared_ptr<boost::thread> sensor_thread = nullptr, tracking_thread = nullptr, calibrate_thread = nullptr,
            imu_thread = nullptr, poseestimation_thread = nullptr, particlefilter_thread = nullptr, distance_thread_1 = nullptr,
            distance_thread_2 = nullptr;
    std::atomic<bool> receiveData, recording;
    string path;
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber sensor_sub;
    int objectID = 0;
    string name = "bastiisdoff";
    string mesh = "pimmel";
    vector<Eigen::Vector3f> object;
    Vector3d origin;
    static bool m_switch;
    ofstream file;
    boost::shared_ptr<UDPSocket> socket;
};

typedef boost::shared_ptr<TrackedObject> TrackedObjectPtr;
