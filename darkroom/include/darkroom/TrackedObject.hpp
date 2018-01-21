#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <roboy_communication_middleware/DarkRoomStatistics.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <deque>
#include <thread>
#include <mutex>
#include <bitset>
#include <common_utilities/rviz_visualization.hpp>
#include <common_utilities/UDPSocket.hpp>
#include "darkroom/LighthouseEstimator.hpp"
#include <robot_localization/ros_filter_types.h>

#define uSecsToRadians(ticks) (degreesToRadians(ticks * 0.021600864))
#define ticksToRadians(ticks) (degreesToRadians(ticks * 0.021600864 / 50.0))
#define ticksToDegrees(ticks) ((ticks * 0.021600864 / 50.0))

using namespace std;

class TrackedObject : public LighthouseEstimator, public RobotLocalization::RosEkf {
public:
    TrackedObject();

    ~TrackedObject();

    /**
     * initializes with yaml config file
     * @param configFile
     * @return success
     */
    bool init(const char* configFile = "calibrationCube.yaml");

    /**
     * Terminates all threads
     */
    void shutDown();

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

    /**
     * regularily publishes the transform between base_link and imu
     */
    void publishImuFrame();

public:
    boost::shared_ptr<boost::thread> sensor_thread = nullptr, tracking_thread = nullptr, calibrate_thread = nullptr,
            imu_thread = nullptr, objectposeestimation_thread = nullptr, poseestimation_thread = nullptr,
            particlefilter_thread = nullptr, distance_thread_1 = nullptr,
            distance_thread_2 = nullptr, relative_pose_thread = nullptr, rays_thread = nullptr;
    std::atomic<bool> receiveData, recording, publish_transform;
    string path;
    string objectID;
private:
    ros::NodeHandlePtr nh;
    ros::Publisher darkroom_statistics_pub;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Subscriber sensor_sub;
    vector<Eigen::Vector3f> object;
    Vector3d origin;
    static bool m_switch;
    ofstream file;
    boost::shared_ptr<UDPSocket> socket;
    boost::shared_ptr<boost::thread> kalman_filter_thread, publish_imu_transform;
    tf::Transform imu;
    tf::TransformBroadcaster tf_broadcaster;
};

typedef boost::shared_ptr<TrackedObject> TrackedObjectPtr;
