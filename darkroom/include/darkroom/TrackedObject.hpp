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
    description: receives sensor data via ROS or UDP, instantiates extended kalman filter for sensor fusion
*/

#pragma once

#include <rclcpp/rclcpp.hpp>
//#include <ros/package.h>
#include <roboy_middleware_msgs/msg/dark_room.hpp>
#include <roboy_middleware_msgs/msg/dark_room_statistics.hpp>
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
#include <robot_localization/ros_filter_types.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

#define uSecsToRadians(ticks) (degreesToRadians(ticks * 0.021600864))
#define ticksToRadians(ticks) (degreesToRadians(ticks * 0.021600864 / 50.0))
#define ticksToDegrees(ticks) ((ticks * 0.021600864 / 50.0))

using namespace std;

class TrackedObject : public LighthouseEstimator{//, public robot_localization::RosEkf {
public:
    TrackedObject(rclcpp::Node::SharedPtr nh);

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
private:

    /**
     * Continuously receiving, decoding and updating the sensor data from ROS message
     */
    void receiveSensorDataRoboy(const roboy_middleware_msgs::msg::DarkRoom::SharedPtr msg);

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
            distance_thread_2 = nullptr, relative_pose_thread = nullptr, relative_pose_epnp_thread = nullptr,
            object_pose_estimation_multi_lighthouse_thread = nullptr,
            rays_thread = nullptr;
    std::atomic<bool> receiveData, recording, publish_transform;
    string path;
    string objectID;
    SharedMutex mux;
private:
    rclcpp::Node::SharedPtr nh;
    std::shared_ptr<rclcpp::Publisher<roboy_middleware_msgs::msg::DarkRoomStatistics>> darkroom_statistics_pub;
    //boost::shared_ptr<ros::AsyncSpinner> spinner;
    std::shared_ptr<rclcpp::Subscription<roboy_middleware_msgs::msg::DarkRoom>> sensor_sub;
    vector<Eigen::Vector3f> object;
    Vector3d origin;
    static bool m_switch;
    ofstream file;
    boost::shared_ptr<UDPSocket> socket;
    boost::shared_ptr<boost::thread> kalman_filter_thread, publish_imu_transform;
    tf2::Transform imu;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    rclcpp::Clock::SharedPtr clock;
    std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
//    robot_localization::RosEkf::SharedPtr ekf;
};

typedef boost::shared_ptr<TrackedObject> TrackedObjectPtr;
