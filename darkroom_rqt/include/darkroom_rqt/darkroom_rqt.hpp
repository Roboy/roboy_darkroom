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
    description: rqt gui plugin for darkroom lighthouse tracking
*/

#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <ros/package.h>
#include <rqt_gui_cpp/plugin.h>
#include <darkroom_rqt/ui_darkroom_rqt.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <darkroom/LighthouseSimulator.hpp>
#include <darkroom/TrackedObject.hpp>
#include <darkroom/Transform.hpp>
#include <roboy_communication_middleware/LighthousePoseCorrection.h>
#include <roboy_communication_middleware/ArucoPose.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <roboy_communication_middleware/DarkRoomStatistics.h>
#include <roboy_communication_middleware/DarkRoomStatus.h>
#include <roboy_communication_middleware/DarkRoomOOTX.h>
#include <map>
#include <QLineEdit>
#include <QSlider>
#include <QPushButton>
#include <common_utilities/rviz_visualization.hpp>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <QFileSystemModel>
#include <QScrollArea>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QLabel>
#include <utility>
#include <QDialog>

#endif

using namespace std;

class RoboyDarkRoom
        : public rqt_gui_cpp::Plugin, rviz_visualization, DarkRoom::Transform {
    Q_OBJECT
public:
    RoboyDarkRoom();
    ~RoboyDarkRoom();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);

public Q_SLOTS:
    /**
     * align tracked object coordinate frame with vive controller
     */
    void alignToViveController();
    /**
     * compare the tracking of a trackedObject that has a vive controller mounted to it
     */
    void compareToSteamVR();
    /**
     * connect Roboy via ROS
     */
    void connectRoboy();
    /**
     * connect Object via UDP socket
     */
    void connectObject();
    /**
     * Clears all visualizations in rviz
     */
    void clearAll();
    /**
     * Resets the lighthouse poses to slider values
     */
    void resetLighthousePoses();
    /**
     * Toggles recording sensor values for all tracked objects
     */
    void record();
    /**
     * Toggles visualization of lighthouse rays
     */
    void showRays();
    /**
     * Toggles visualization of relative sensor distances
     */
    void showDistances();
    /**
     * Switches IDs of lighthouses
     */
    void switchLighthouses();
    /**
     * Toggles tracking thread
     * @param start toggle flag
     */
    void startTriangulation();
    /**
     * Toggles calibration thread
     */
    void startCalibrateRelativeSensorDistances();
    /**
     * Toggles poseestimation thread
     */
    void startPoseEstimationSensorCloud();
    /**
     * Toggles object poseestimation thread
     */
    void startObjectPoseEstimationSensorCloud();
    /**
     * Toggles distance estimation thread
     */
    void startEstimateSensorPositionsUsingRelativeDistances();
    /**
     * Toggles relative pose estimation thread
     */
    void startEstimateObjectPoseUsingRelativeDistances();
    /**
     * Toggles relative pose estimation epnp thread
     */
    void startEstimateObjectPoseEPNP();
    /**
     * Toggles object pose estimation using multi lighthouse approach
     */
    void startEstimateObjectPoseMultiLighthouse();
    /**
     * Toggles least squares pose estimation thread
     */
    void startObjectPoseEstimationLeastSquares();
    /**
     * Plots the data
     */
    void plotData();
    /**
     * Plots statistics data
     */
    void plotStatisticsData();
    /**
     * Toggles usage of factory calibration data phase
     */
    void useFactoryCalibrationData();
    /**
     * adds a tracked object
     * @param config_file_path path to yaml config file
     * @return success
     */
    bool addTrackedObject(const char* config_file_path = "");
    /**
     * removes the selected tracked object
     */
    void removeTrackedObject();
    /**
     * Updates calibration values
     */
    void updateCalibrationValues();
    /**
     * Use Vive calibration values
     */
    void useViveCalibrationValues();
    /**
     * Estimates the factory calibration values
     */
    void estimateFactoryCalibration();

    /**
     * Estimates the factory calibration values
     */
    void estimateFactoryCalibration2();

    /**
     * Estimates the factory calibration values using epnp
     */
    void estimateFactoryCalibrationEPNP();

    /**
     * Estimates the factory calibration values using multi lighthouse pose estimation
     */
    void estimateFactoryCalibrationMulti();

    /**
     * Resets all tracked objects calibration values
     */
    void resetFactoryCalibration();
    /**
     * Resets all tracked objects poses
     */
    void resetPose();
private:
    /**
     * Is regularily publishing the tf frames (lighthouse1, lighthouse2)
     */
    void transformPublisher();

    /**
     * Callback for pose correction message
     * @param msg
     */
    void correctPose(const roboy_communication_middleware::LighthousePoseCorrection &msg);

    /**
     * Callback for interactive markers
     * @param msg
     */
    void interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg);

    /**
     * Callback for DarkRoom data
     */
    void receiveSensorData(const roboy_communication_middleware::DarkRoom::ConstPtr &msg);
    /**
     * Callback for DarkRoom sensor status
     */
    void receiveSensorStatus(const roboy_communication_middleware::DarkRoomStatus::ConstPtr &msg);
    /**
     * Callback for DarkRoom statistics
     * @param msg
     */
    void receiveStatistics(const roboy_communication_middleware::DarkRoomStatistics::ConstPtr &msg);
    /**
     * Callback for DarkRoom ootx
     * @param msg
     */
    void receiveOOTXData(const roboy_communication_middleware::DarkRoomOOTX::ConstPtr &msg);
    /**
     * Checks if a file exists
     * @param filepath
     * @return exists
     */
    inline bool fileExists(const string &filepath);

    void updateTrackedObjectInfo();

    void receiveArucoPose(const roboy_communication_middleware::ArucoPose::ConstPtr &msg);
Q_SIGNALS:
    void newData();
    void newStatisticsData();
private:
    Ui::RoboyDarkRoom ui;
    QWidget *widget_;

    QVector<double> time[4], horizontal_angle[2], vertical_angle[2], statistics_time[2];
    map<int, QVector<double>[2]> updateFrequencies[2];
    int counter = 0;
    QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    boost::shared_ptr<std::thread> transform_thread = nullptr, update_tracked_object_info_thread = nullptr;
    ros::Subscriber pose_correction_sub, interactive_marker_sub, sensor_sub, sensor_status_sub, statistics_sub, ootx_sub, aruco_pose_sub;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    static tf::Transform lighthouse1, lighthouse2, tf_world, tf_map,
            simulated_object_lighthouse1, simulated_object_lighthouse2;
    atomic<bool> publish_transform, update_tracked_object_info;
    int object_counter = 0, values_in_plot = 300, message_counter[4] = {0}, message_counter_statistics[2] = {0};
    vector<TrackedObjectPtr> trackedObjects;
    vector<string> trackedObjectsIDs;
    SharedMutex mux;
    static map<string, QLineEdit*> text;
    static map<string, QSlider*> slider;
    static map<string, QPushButton*> button;
    bool simulate = false;
    float random_pose_x = 0, random_pose_y = 0, random_pose_z = 0, random_pose_roll = 0, random_pose_pitch = 0, random_pose_yaw = 0;
    vector<pair<LighthouseSimulatorPtr,LighthouseSimulatorPtr>> lighthouse_simulation;

    QFileSystemModel *model;
    struct TrackedObjectInfo{
        QWidget *widget;
        QCheckBox *selected;
        QLabel* name;
        QLabel* activeSensors;
    };
    vector<TrackedObjectInfo> trackedObjectsInfo;
    map<int,Vector3d> aruco_position_mean;
    map<int,Vector3d> aruco_position_variance;
    map<int,long> receive_counter;
    LighthouseCalibration calibration[2][2];
};
