#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <darkroom_rqt/ui_darkroom_rqt.h>
#include <QWidget>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <darkroom/LighthouseSimulator.hpp>
#include <darkroom/TrackedObject.hpp>
#include <darkroom/Transform.hpp>
#include <roboy_communication_middleware/LighthousePoseCorrection.h>
#include <roboy_communication_middleware/DarkRoom.h>
#include <roboy_communication_middleware/DarkRoomStatistics.h>
#include <roboy_communication_middleware/DarkRoomOOTX.h>
#include <map>
#include <QLineEdit>
#include <QSlider>
#include <QPushButton>
#include <common_utilities/rviz_visualization.hpp>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#endif

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
     * connect Roboy via ROS
     */
    void connectRoboy();

    /**
     * connect simulated Roboy via ROS
     */
    void simulateRoboy();

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
     * Toggles particle filter thread for pose correction
     */
    void startPoseEstimationParticleFilter();
    /**
     * Toggles least squares pose estimation thread
     */
    void startObjectPoseEstimationLeastSquares();
    /**
     * Toggles epnp pose estimation thread
     */
    void startPoseEstimationEPnP();
    /**
     * Toggles p3p pose estimation thread
     */
    void startPoseEstimationP3P();
    /**
     * Loads an object (updateing calibrated sensor distances)
     */
    void loadObject();
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
     * Callback for DarkRoom statistics
     * @param msg
     */
    void receiveStatistics(const roboy_communication_middleware::DarkRoomStatistics::ConstPtr &msg);
    /**
     * Callback for DarkRoom ootx
     * @param msg
     */
    void receiveOOTXData(const roboy_communication_middleware::DarkRoomOOTX::ConstPtr &msg);
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
    boost::shared_ptr<std::thread> transform_thread = nullptr;
    ros::Subscriber pose_correction_sub, interactive_marker_sub, sensor_sub, statistics_sub, ootx_sub;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    static tf::Transform lighthouse1, lighthouse2, tf_world, tf_map,
            simulated_object_lighthouse1, simulated_object_lighthouse2;
    atomic<bool> publish_transform;
    int object_counter = 0, values_in_plot = 300, message_counter[4] = {0}, message_counter_statistics[2] = {0};
    map<int, TrackedObjectPtr> trackedObjects;
    mutex mux;
    static map<string, QLineEdit*> text;
    static map<string, QSlider*> slider;
    static map<string, QPushButton*> button;
    bool simulate = false;

    map<int, boost::shared_ptr<LighthouseSimulator>> lighthouse_simulation;
};
