#include <darkroom_rqt/darkroom_rqt.hpp>

tf::Transform RoboyDarkRoom::lighthouse1;
tf::Transform RoboyDarkRoom::lighthouse2;
tf::Transform RoboyDarkRoom::simulated_object_lighthouse1;
tf::Transform RoboyDarkRoom::simulated_object_lighthouse2;
tf::Transform RoboyDarkRoom::tf_world;
tf::Transform RoboyDarkRoom::tf_map;

map<string, QLineEdit *> RoboyDarkRoom::text;
map<string, QPushButton *> RoboyDarkRoom::button;
map<string, QSlider *> RoboyDarkRoom::slider;

RoboyDarkRoom::RoboyDarkRoom()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyDarkRoom");
}

RoboyDarkRoom::~RoboyDarkRoom() {
    publish_transform = false;
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object.second->mux);
        object.second->calibrating = false;
        object.second->distances = false;
        object.second->particle_filtering = false;
        object.second->poseestimating = false;
        object.second->tracking = false;
    }
}

void RoboyDarkRoom::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    text["lighthouse1_x"] = widget_->findChild<QLineEdit *>("lighthouse1_x_text");
    text["lighthouse1_y"] = widget_->findChild<QLineEdit *>("lighthouse1_y_text");
    text["lighthouse1_z"] = widget_->findChild<QLineEdit *>("lighthouse1_z_text");
    text["lighthouse1_roll"] = widget_->findChild<QLineEdit *>("lighthouse1_roll_text");
    text["lighthouse1_pitch"] = widget_->findChild<QLineEdit *>("lighthouse1_pitch_text");
    text["lighthouse1_yaw"] = widget_->findChild<QLineEdit *>("lighthouse1_yaw_text");

    text["lighthouse2_x"] = widget_->findChild<QLineEdit *>("lighthouse2_x_text");
    text["lighthouse2_y"] = widget_->findChild<QLineEdit *>("lighthouse2_y_text");
    text["lighthouse2_z"] = widget_->findChild<QLineEdit *>("lighthouse2_z_text");
    text["lighthouse2_roll"] = widget_->findChild<QLineEdit *>("lighthouse2_roll_text");
    text["lighthouse2_pitch"] = widget_->findChild<QLineEdit *>("lighthouse2_pitch_text");
    text["lighthouse2_yaw"] = widget_->findChild<QLineEdit *>("lighthouse2_yaw_text");

    text["simulatedObject_x_lighthouse_1"] = widget_->findChild<QLineEdit *>("simulatedObject_x_text_lighthouse_1");
    text["simulatedObject_y_lighthouse_1"] = widget_->findChild<QLineEdit *>("simulatedObject_y_text_lighthouse_1");
    text["simulatedObject_z_lighthouse_1"] = widget_->findChild<QLineEdit *>("simulatedObject_z_text_lighthouse_1");
    text["simulatedObject_roll_lighthouse_1"] = widget_->findChild<QLineEdit *>(
            "simulatedObject_roll_text_lighthouse_1");
    text["simulatedObject_pitch_lighthouse_1"] = widget_->findChild<QLineEdit *>(
            "simulatedObject_pitch_text_lighthouse_1");
    text["simulatedObject_yaw_lighthouse_1"] = widget_->findChild<QLineEdit *>("simulatedObject_yaw_text_lighthouse_1");

    text["simulatedObject_x_lighthouse_2"] = widget_->findChild<QLineEdit *>("simulatedObject_x_text_lighthouse_2");
    text["simulatedObject_y_lighthouse_2"] = widget_->findChild<QLineEdit *>("simulatedObject_y_text_lighthouse_2");
    text["simulatedObject_z_lighthouse_2"] = widget_->findChild<QLineEdit *>("simulatedObject_z_text_lighthouse_2");
    text["simulatedObject_roll_lighthouse_2"] = widget_->findChild<QLineEdit *>(
            "simulatedObject_roll_text_lighthouse_2");
    text["simulatedObject_pitch_lighthouse_2"] = widget_->findChild<QLineEdit *>(
            "simulatedObject_pitch_text_lighthouse_2");
    text["simulatedObject_yaw_lighthouse_2"] = widget_->findChild<QLineEdit *>("simulatedObject_yaw_text_lighthouse_2");

    text["object_x"] = widget_->findChild<QLineEdit *>("object_x_text");
    text["object_y"] = widget_->findChild<QLineEdit *>("object_y_text");
    text["object_z"] = widget_->findChild<QLineEdit *>("object_z_text");
    text["object_roll"] = widget_->findChild<QLineEdit *>(
            "object_roll_text");
    text["object_pitch"] = widget_->findChild<QLineEdit *>(
            "object_pitch_text");
    text["object_yaw"] = widget_->findChild<QLineEdit *>("object_yaw_text");

    text["object_pose_estimate_x"] = widget_->findChild<QLineEdit *>("object_pose_estimate_x_text");
    text["object_pose_estimate_y"] = widget_->findChild<QLineEdit *>("object_pose_estimate_y_text");
    text["object_pose_estimate_z"] = widget_->findChild<QLineEdit *>("object_pose_estimate_z_text");
    text["object_pose_estimate_roll"] = widget_->findChild<QLineEdit *>(
            "object_pose_estimate_roll_text");
    text["object_pose_estimate_pitch"] = widget_->findChild<QLineEdit *>(
            "object_pose_estimate_pitch_text");
    text["object_pose_estimate_yaw"] = widget_->findChild<QLineEdit *>("object_pose_estimate_yaw_text");

    text["broadcast_ip"] = widget_->findChild<QLineEdit *>("broadcast_ip");
    text["broadcast_port"] = widget_->findChild<QLineEdit *>("broadcast_port");
    text["load_object"] = widget_->findChild<QLineEdit *>("load_object_text");

    button["triangulate"] = widget_->findChild<QPushButton *>("triangulate");
    button["show_rays"] = widget_->findChild<QPushButton *>("show_rays");
    button["show_distances"] = widget_->findChild<QPushButton *>("show_distances");
    button["pose_correction_least_squares"] = widget_->findChild<QPushButton *>("pose_correction_least_squares");
    button["pose_correction_particle_filter"] = widget_->findChild<QPushButton *>("pose_correction_particle_filter");
    button["position_estimation_relativ_sensor_distances"] = widget_->findChild<QPushButton *>(
            "position_estimation_relativ_sensor_distances");
    button["reset_lighthouse_poses"] = widget_->findChild<QPushButton *>("reset_lighthouse_poses");
    button["switch_lighthouses"] = widget_->findChild<QPushButton *>("switch_lighthouses");
    button["calibrate_relative_distances"] = widget_->findChild<QPushButton *>("calibrate_relative_distances");
    button["record"] = widget_->findChild<QPushButton *>("record");
    button["connect_roboy"] = widget_->findChild<QPushButton *>("connect_roboy");
    button["simulate_roboy"] = widget_->findChild<QPushButton *>("simulate_roboy");
    button["connect_object"] = widget_->findChild<QPushButton *>("connect_object");
    button["clear_all"] = widget_->findChild<QPushButton *>("clear_all");
    button["object_pose_estimation_least_squares"] = widget_->findChild<QPushButton *>(
            "object_pose_estimation_least_squares");
    button["load_object"] = widget_->findChild<QPushButton *>("load_object");

    button["triangulate"]->setToolTip(
            "activates triangulation of lighthouse rays\n"
                    "to get the 3d Position of visible sensors");
    button["show_rays"]->setToolTip("enable to visualize the lighthouse rays");
    button["show_distances"]->setToolTip("enable to visualize the distances\nbetween triangulated sensor positions");
    button["pose_correction_least_squares"]->setToolTip(
            "use the known relative distances between\nsensors to estimate the distances to\neach lighthouse. "
                    "Then run a pose minimizer\nto match these sensor locations onto each other. ");
    button["position_estimation_relativ_sensor_distances"]->setToolTip(
            "Use the known relative distances between sensors to estimate the distances to each lighthouse.");
    button["reset_lighthouse_poses"]->setToolTip("reset the lighthouse poses\nto the slider values");
    button["switch_lighthouses"]->setToolTip("switch lighthouses");
    button["calibrate_relative_distances"]->setToolTip("calibrate the relative distances\nof an unknown object "
                                                               "by averaging the triangulated\npositions for a couple of seconds."
                                                               "The result is written into a\nconfig file in calibrated_objects");
    button["record"]->setToolTip("records the sensor data to a log file");
    button["connect_roboy"]->setToolTip("subscribe to DarkRoom sensory data via ROS message");
    button["connect_object"]->setToolTip("subscribe to DarkRoom sensory data via UDP message");
    button["simulate_roboy"]->setToolTip("simulate lighthouse data");
    button["clear_all"]->setToolTip("clears all visualization markers");
    button["object_pose_estimation_least_squares"]->setToolTip("estimates object pose using least squares minimizer");
    button["load_object"]->setToolTip("loads the given object yaml file");

    QObject::connect(button["triangulate"], SIGNAL(clicked()), this, SLOT(startTriangulation()));
    QObject::connect(button["show_rays"], SIGNAL(clicked()), this, SLOT(showRays()));
    QObject::connect(button["show_distances"], SIGNAL(clicked()), this, SLOT(showDistances()));
    QObject::connect(button["pose_correction_least_squares"], SIGNAL(clicked()), this,
                     SLOT(startPoseEstimationSensorCloud()));
    QObject::connect(button["position_estimation_relativ_sensor_distances"], SIGNAL(clicked()), this,
                     SLOT(startEstimateSensorPositionsUsingRelativeDistances()));
    QObject::connect(button["reset_lighthouse_poses"], SIGNAL(clicked()), this, SLOT(resetLighthousePoses()));
    QObject::connect(button["calibrate_relative_distances"], SIGNAL(clicked()), this,
                     SLOT(startCalibrateRelativeSensorDistances()));
    QObject::connect(button["switch_lighthouses"], SIGNAL(clicked()), this, SLOT(switchLighthouses()));
    QObject::connect(button["record"], SIGNAL(clicked()), this, SLOT(record()));
    QObject::connect(button["connect_roboy"], SIGNAL(clicked()), this, SLOT(connectRoboy()));
    QObject::connect(button["simulate_roboy"], SIGNAL(clicked()), this, SLOT(simulateRoboy()));
    QObject::connect(button["connect_object"], SIGNAL(clicked()), this, SLOT(connectObject()));
    QObject::connect(button["clear_all"], SIGNAL(clicked()), this, SLOT(clearAll()));
    QObject::connect(button["object_pose_estimation_least_squares"], SIGNAL(clicked()), this,
                     SLOT(startObjectPoseEstimationSensorCloud()));
    QObject::connect(button["load_object"], SIGNAL(clicked()), this, SLOT(loadObject()));

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(this, SIGNAL(newStatisticsData()), this, SLOT(plotStatisticsData()));


    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "darkroom_rqt_plugin");
    }

    pose_correction_sub = nh->subscribe("/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1,
                                        &RoboyDarkRoom::correctPose, this);
    sensor_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensors", 1, &RoboyDarkRoom::receiveSensorData, this);
    statistics_sub = nh->subscribe("/roboy/middleware/DarkRoom/Statistics", 2, &RoboyDarkRoom::receiveStatistics, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    resetLighthousePoses();

    publish_transform = true;
    if (transform_thread == nullptr) {
        transform_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyDarkRoom::transformPublisher, this));
        transform_thread->detach();
    }

    interactive_marker_sub = nh->subscribe("/interactive_markers/feedback", 1,
                                           &RoboyDarkRoom::interactiveMarkersFeedback, this);

    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, lighthouse1.getOrigin(),
                   true, 0.1, "world", "lighthouse1", "");
    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, lighthouse2.getOrigin(),
                   true, 0.1, "world", "lighthouse2", "");

    ui.horizontal_angle_lighthouse_1->addGraph();
    ui.horizontal_angle_lighthouse_1->graph(0)->setPen(QPen(color_pallette[0]));
    ui.horizontal_angle_lighthouse_1->yAxis->setLabel("degrees");
    ui.horizontal_angle_lighthouse_1->yAxis->setRange(0, 180);

    ui.horizontal_angle_lighthouse_2->addGraph();
    ui.horizontal_angle_lighthouse_2->graph(0)->setPen(QPen(color_pallette[0]));
    ui.horizontal_angle_lighthouse_2->yAxis->setLabel("degrees");
    ui.horizontal_angle_lighthouse_2->yAxis->setRange(0, 180);

    ui.vertical_angle_lighthouse_1->addGraph();
    ui.vertical_angle_lighthouse_1->graph(0)->setPen(QPen(color_pallette[1]));
    ui.vertical_angle_lighthouse_1->yAxis->setLabel("degrees");
    ui.vertical_angle_lighthouse_1->yAxis->setRange(0, 180);

    ui.vertical_angle_lighthouse_2->addGraph();
    ui.vertical_angle_lighthouse_2->graph(0)->setPen(QPen(color_pallette[1]));
    ui.vertical_angle_lighthouse_2->yAxis->setLabel("degrees");
    ui.vertical_angle_lighthouse_2->yAxis->setRange(0, 180);

    for(uint i=0; i<32; i++){
        QColor color(((float)rand()/RAND_MAX)*255,((float)rand()/RAND_MAX)*255,((float)rand()/RAND_MAX)*255);
        ui.update_frequencies_horizontal_lighthouse_1->addGraph();
        ui.update_frequencies_horizontal_lighthouse_1->graph(i)->setPen(QPen(color));
        ui.update_frequencies_horizontal_lighthouse_2->addGraph();
        ui.update_frequencies_horizontal_lighthouse_2->graph(i)->setPen(QPen(color));
        ui.update_frequencies_vertical_lighthouse_1->addGraph();
        ui.update_frequencies_vertical_lighthouse_1->graph(i)->setPen(QPen(color));
        ui.update_frequencies_vertical_lighthouse_2->addGraph();
        ui.update_frequencies_vertical_lighthouse_2->graph(i)->setPen(QPen(color));
    }
    ui.update_frequencies_horizontal_lighthouse_1->yAxis->setLabel("frequency[Hz]");
    ui.update_frequencies_horizontal_lighthouse_1->yAxis->setRange(0, 1000);
    ui.update_frequencies_horizontal_lighthouse_2->yAxis->setLabel("frequency[Hz]");
    ui.update_frequencies_horizontal_lighthouse_2->yAxis->setRange(0, 1000);
    ui.update_frequencies_vertical_lighthouse_1->yAxis->setLabel("frequency[Hz]");
    ui.update_frequencies_vertical_lighthouse_1->yAxis->setRange(0, 1000);
    ui.update_frequencies_vertical_lighthouse_2->yAxis->setLabel("frequency[Hz]");
    ui.update_frequencies_vertical_lighthouse_2->yAxis->setRange(0, 1000);
}

void RoboyDarkRoom::shutdownPlugin() {
}

void RoboyDarkRoom::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                 qt_gui_cpp::Settings &instance_settings) const {
    instance_settings.setValue("lighthouse1_x", text["lighthouse1_x"]->text());
    instance_settings.setValue("lighthouse1_y", text["lighthouse1_y"]->text());
    instance_settings.setValue("lighthouse1_z", text["lighthouse1_z"]->text());
    instance_settings.setValue("lighthouse2_x", text["lighthouse2_x"]->text());
    instance_settings.setValue("lighthouse2_y", text["lighthouse2_y"]->text());
    instance_settings.setValue("lighthouse2_z", text["lighthouse2_z"]->text());
}

void RoboyDarkRoom::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                    const qt_gui_cpp::Settings &instance_settings) {
    text["load_object"]->setText(instance_settings.value("load_object").toString());
    text["lighthouse1_x"]->setText(instance_settings.value("lighthouse1_x").toString());
    text["lighthouse1_y"]->setText(instance_settings.value("lighthouse1_y").toString());
    text["lighthouse1_z"]->setText(instance_settings.value("lighthouse1_z").toString());
    text["lighthouse2_x"]->setText(instance_settings.value("lighthouse2_x").toString());
    text["lighthouse2_y"]->setText(instance_settings.value("lighthouse2_y").toString());
    text["lighthouse2_z"]->setText(instance_settings.value("lighthouse2_z").toString());
}

void RoboyDarkRoom::connectRoboy() {
    ROS_DEBUG("connect roboy clicked");
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->connectRoboy();
    trackedObjects[object_counter] = newObject;
    object_counter++;
    resetObjectPoses();
}

void RoboyDarkRoom::simulateRoboy() {
    ROS_DEBUG("simulate roboy clicked");

    resetObjectPoses();

    simulate = true;

    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->connectRoboy();
    trackedObjects[object_counter] = newObject;
    object_counter++;

    lighthouse_simulation[LIGHTHOUSE_A].reset(new LighthouseSimulator(LIGHTHOUSE_A));
    lighthouse_simulation[LIGHTHOUSE_B].reset(new LighthouseSimulator(LIGHTHOUSE_B));


    lighthouse_simulation[LIGHTHOUSE_A]->sensor_publishing = true;
    lighthouse_simulation[LIGHTHOUSE_B]->sensor_publishing = true;

    lighthouse_simulation[LIGHTHOUSE_A]->sensor_thread = boost::shared_ptr<boost::thread>(
            new boost::thread(
                    [this]() { this->lighthouse_simulation[LIGHTHOUSE_A]->PublishSensorData(); }
            ));
    lighthouse_simulation[LIGHTHOUSE_B]->sensor_thread = boost::shared_ptr<boost::thread>(
            new boost::thread(
                    [this]() { this->lighthouse_simulation[LIGHTHOUSE_B]->PublishSensorData(); }
            ));

    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, tf::Vector3(0, 0, 0),
                   true, 0.1, "world", "trackedObject", "");
}

void RoboyDarkRoom::connectObject() {
    ROS_DEBUG("connect object clicked");
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    bool ok;
    newObject->connectObject(text["broadcast_ip"]->text().toStdString().c_str(),
                             text["broadcast_port"]->text().toInt(&ok));
    trackedObjects[object_counter] = newObject;
    object_counter++;
}

void RoboyDarkRoom::clearAll() {
    ROS_DEBUG("clear all clicked");
    for (auto const &object:trackedObjects) {
        object.second->clearAll();
    }
}

void RoboyDarkRoom::resetLighthousePoses() {
    ROS_DEBUG("reset lighthouse poses");
    tf_world.setOrigin(tf::Vector3(0, 0, 0));
    tf_map.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tf_world.setRotation(quat);
    tf_map.setRotation(quat);
    bool ok;
    quat.setRPY(
            text["lighthouse1_roll"]->text().toFloat(&ok) * M_PI / 180.0,
            text["lighthouse1_pitch"]->text().toFloat(&ok) * M_PI / 180.0,
            text["lighthouse1_yaw"]->text().toFloat(&ok) * M_PI / 180.0
    );
    lighthouse1.setRotation(quat);
    lighthouse1.setOrigin(tf::Vector3(
            text["lighthouse1_x"]->text().toFloat(&ok),
            text["lighthouse1_y"]->text().toFloat(&ok),
            text["lighthouse1_z"]->text().toFloat(&ok)
    ));

    quat.setRPY(
            text["lighthouse2_roll"]->text().toFloat(&ok) * M_PI / 180.0,
            text["lighthouse2_pitch"]->text().toFloat(&ok) * M_PI / 180.0,
            text["lighthouse2_yaw"]->text().toFloat(&ok) * M_PI / 180.0
    );
    lighthouse2.setRotation(quat);
    lighthouse2.setOrigin(tf::Vector3(
            text["lighthouse2_x"]->text().toFloat(&ok),
            text["lighthouse2_y"]->text().toFloat(&ok),
            text["lighthouse2_z"]->text().toFloat(&ok)
    ));
}

void RoboyDarkRoom::resetObjectPoses() {
    ROS_DEBUG("reset object poses");

    tf::Quaternion quat;
    bool ok;
    quat.setRPY(
            text["simulatedObject_roll_lighthouse_1"]->text().toFloat(&ok) * M_PI / 180.0,
            text["simulatedObject_pitch_lighthouse_1"]->text().toFloat(&ok) * M_PI / 180.0,
            text["simulatedObject_yaw_lighthouse_1"]->text().toFloat(&ok) * M_PI / 180.0
    );
    simulated_object_lighthouse1.setRotation(quat);
    simulated_object_lighthouse1.setOrigin(tf::Vector3(
            text["simulatedObject_x_lighthouse_1"]->text().toFloat(&ok),
            text["simulatedObject_y_lighthouse_1"]->text().toFloat(&ok),
            text["simulatedObject_z_lighthouse_1"]->text().toFloat(&ok)
    ));

    quat.setRPY(
            text["simulatedObject_roll_lighthouse_2"]->text().toFloat(&ok) * M_PI / 180.0,
            text["simulatedObject_pitch_lighthouse_2"]->text().toFloat(&ok) * M_PI / 180.0,
            text["simulatedObject_yaw_lighthouse_2"]->text().toFloat(&ok) * M_PI / 180.0
    );
    simulated_object_lighthouse2.setRotation(quat);
    simulated_object_lighthouse2.setOrigin(tf::Vector3(
            text["simulatedObject_x_lighthouse_2"]->text().toFloat(&ok),
            text["simulatedObject_y_lighthouse_2"]->text().toFloat(&ok),
            text["simulatedObject_z_lighthouse_2"]->text().toFloat(&ok)
    ));
}

void RoboyDarkRoom::record() {
    ROS_DEBUG("record clicked");
    if (trackedObjects.empty())
        button["record"]->setChecked(false);
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object.second->mux);
        object.second->record(button["record"]->isChecked());
    }
}

void RoboyDarkRoom::showRays() {
    ROS_DEBUG("show rays clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object.second->mux);
        object.second->rays = button["show_rays"]->isChecked();
    }
}

void RoboyDarkRoom::showDistances() {
    ROS_DEBUG("show distances clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object.second->mux);
        object.second->distances = object.second->rays = button["show_distances"]->isChecked();
    }
}

void RoboyDarkRoom::switchLighthouses() {
    ROS_DEBUG("switch lighthouses clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object.second->mux);
        object.second->switchLighthouses(button["switch_lighthouses"]->isChecked());
    }
}

void RoboyDarkRoom::startCalibrateRelativeSensorDistances() {
    ROS_DEBUG("calibrate_relative_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        if (button["calibrate_relative_distances"]->isChecked()) {
            ROS_INFO("starting calibration thread");
            trackedObjects[i]->calibrating = true;
            trackedObjects[i]->calibrate_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread(
                            [this, i]() { this->trackedObjects[i]->calibrateRelativeSensorDistances(); }
                    ));
            trackedObjects[i]->calibrate_thread->detach();
        } else {
            if (trackedObjects[i]->calibrate_thread != nullptr) {
                trackedObjects[i]->calibrating = false;
                if (trackedObjects[i]->calibrate_thread->joinable()) {
                    ROS_INFO("Waiting for calibration thread to terminate");
                    trackedObjects[i]->calibrate_thread->join();
                }
            }
        }
    }
}

void RoboyDarkRoom::startTriangulation() {
    ROS_DEBUG("triangulate clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        if (button["triangulate"]->isChecked()) {
            ROS_INFO("starting tracking thread");
            trackedObjects[i]->tracking = true;
            trackedObjects[i]->tracking_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread(
                            [this, i]() { this->trackedObjects[i]->triangulateSensors(); }
                    ));
            trackedObjects[i]->tracking_thread->detach();
        } else {
            if (trackedObjects[i]->tracking_thread != nullptr) {
                ROS_INFO("stopping tracking thread");
                trackedObjects[i]->tracking = false;
                if (trackedObjects[i]->tracking_thread->joinable()) {
                    ROS_INFO_THROTTLE(1,"Waiting for tracking thread to terminate");
                    trackedObjects[i]->tracking_thread->join();
                }
            }
        }
    }
}

void RoboyDarkRoom::startPoseEstimationSensorCloud() {
    ROS_DEBUG("pose_correction_sensor_cloud clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        ROS_INFO("starting pose estimation thread");
        trackedObjects[i]->poseestimating = true;
        trackedObjects[i]->poseestimation_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->lighthousePoseEstimationLeastSquares();
                }));
        trackedObjects[i]->poseestimation_thread->detach();
    }
}

void RoboyDarkRoom::startObjectPoseEstimationSensorCloud() {
    ROS_DEBUG("object pose estimation clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        if (button["object_pose_estimation_least_squares"]->isChecked()) {
            ROS_INFO("starting pose estimation thread");
            trackedObjects[i]->objectposeestimating = true;
            trackedObjects[i]->objectposeestimation_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread([this, i]() {
                        this->trackedObjects[i]->objectPoseEstimationLeastSquares();
                    }));
            trackedObjects[i]->objectposeestimation_thread->detach();
        } else {
            if (trackedObjects[i]->objectposeestimation_thread != nullptr) {
                ROS_INFO("stopping pose estimation thread");
                trackedObjects[i]->objectposeestimating = false;
                if (trackedObjects[i]->objectposeestimation_thread->joinable()) {
                    ROS_INFO_THROTTLE(1,"Waiting for pose estimation thread to terminate");
                    trackedObjects[i]->objectposeestimation_thread->join();
                }
            }
        }
    }
}

void RoboyDarkRoom::startEstimateSensorPositionsUsingRelativeDistances() {
    ROS_DEBUG("position_estimation_relativ_sensor_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        ROS_INFO("starting relativ distance thread for lighthouse 1");
        trackedObjects[i]->distance_thread_1 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A);
                }));
        trackedObjects[i]->distance_thread_1->detach();
        ROS_INFO("starting relativ distance thread for lighthouse 2");
        trackedObjects[i]->distance_thread_2 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B);
                }));
        trackedObjects[i]->distance_thread_2->detach();
    }
}

void RoboyDarkRoom::startPoseEstimationParticleFilter() {
    ROS_DEBUG("pose_correction_particle_filter clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        ROS_INFO("starting particle filter thread");
        trackedObjects[i]->particle_filtering = true;
        trackedObjects[i]->particlefilter_thread = boost::shared_ptr<boost::thread>(
                new boost::thread(
                        [this, i]() { this->trackedObjects[i]->poseEstimationParticleFilter(); }
                ));
        trackedObjects[i]->particlefilter_thread->detach();
    }
}

void RoboyDarkRoom::startPoseEstimationEPnP() {
    ROS_DEBUG("object_pose_estimation_epnp clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        ROS_INFO("starting epnp pose estimation thread");
        trackedObjects[i]->particle_filtering = true;
        trackedObjects[i]->particlefilter_thread = boost::shared_ptr<boost::thread>(
                new boost::thread(
                        [this, i]() { this->trackedObjects[i]->poseEstimationEPnP(); }
                ));
        trackedObjects[i]->particlefilter_thread->detach();
    }
}

void RoboyDarkRoom::startPoseEstimationP3P() {
    ROS_DEBUG("object_pose_estimation_p3p clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        ROS_INFO("starting p3p pose estimation thread");
        trackedObjects[i]->particle_filtering = true;
        trackedObjects[i]->particlefilter_thread = boost::shared_ptr<boost::thread>(
                new boost::thread(
                        [this, i]() { this->trackedObjects[i]->poseEstimationP3P(); }
                ));
        trackedObjects[i]->particlefilter_thread->detach();
    }
}

void RoboyDarkRoom::loadObject() {
    ROS_DEBUG("load_object clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
//        trackedObjects[i]->readConfig(trackedObjects[i]->path + "/" + text["load_object"]->text().toStdString());
    }
}

void RoboyDarkRoom::transformPublisher() {
    ros::Rate rate(30);
    while (publish_transform) {
        lock_guard<mutex> lock(mux);
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
        for (auto &simulated:lighthouse_simulation) {
            tf_broadcaster.sendTransform(tf::StampedTransform(simulated.second->relative_object_pose, ros::Time::now(),
                                                              (simulated.second->id == 0 ? "lighthouse1"
                                                                                         : "lighthouse2"),
                                                              simulated.second->name.c_str()));
        }
        for (auto &object:trackedObjects) {
            tf_broadcaster.sendTransform(tf::StampedTransform(object.second->pose, ros::Time::now(),
                                                              "world", object.second->name.c_str()));
        }
        rate.sleep();
    }
}

void RoboyDarkRoom::correctPose(const roboy_communication_middleware::LighthousePoseCorrection &msg) {
    tf::Transform tf;
    tf::transformMsgToTF(msg.tf, tf);
    if (msg.id == LIGHTHOUSE_A) {
        if (msg.type == 0) // relativ
            lighthouse1 = tf * lighthouse1;
        else    // absolut
            lighthouse1 = tf;
    } else {
        if (msg.type == 0) // relativ
            lighthouse2 = tf * lighthouse2;
        else    // absolut
            lighthouse2 = tf;
    }
}

void RoboyDarkRoom::interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg) {
    tf::Vector3 position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    tf::Quaternion orientation(msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w);
    if (strcmp(msg.marker_name.c_str(), "lighthouse1") == 0) {
        lighthouse1.setOrigin(position);
        lighthouse1.setRotation(orientation);
        text["lighthouse1_x"]->setText(QString::number(position.x()));
        text["lighthouse1_y"]->setText(QString::number(position.y()));
        text["lighthouse1_z"]->setText(QString::number(position.z()));
    } else if (strcmp(msg.marker_name.c_str(), "lighthouse2") == 0) {
        lighthouse2.setOrigin(position);
        lighthouse2.setRotation(orientation);
        text["lighthouse2_x"]->setText(QString::number(position.x()));
        text["lighthouse2_y"]->setText(QString::number(position.y()));
        text["lighthouse2_z"]->setText(QString::number(position.z()));
    } else if (strcmp(msg.marker_name.c_str(), "trackedObject") == 0) {
        for (auto &object:trackedObjects) {
            object.second->pose.setOrigin(position);
            object.second->pose.setRotation(orientation);
            for (auto &simulated:lighthouse_simulation) {
                tf::Transform tf;
                if (!getTransform(object.second->name.c_str(),
                                  (simulated.second->id ? "lighthouse2" : "lighthouse1"), tf))
                    continue;
                simulated.second->relative_object_pose = tf;
            }
        }

    }
}

void RoboyDarkRoom::receiveSensorData(const roboy_communication_middleware::DarkRoom::ConstPtr &msg){
    ROS_DEBUG_THROTTLE(10, "receiving sensor data");
    uint id = 0;
    uint lighthouse, rotor, sweepDuration;
    for (uint32_t const &data:msg->sensor_value) {
        if(ui.sensor_selector->value() == id) {
            lighthouse = (data >> 31) & 0x1;
            rotor = (data >> 30) & 0x1;
            int valid = (data >> 29) & 0x1;
            sweepDuration = (data & 0x1fffffff); // raw sensor duration is 50 ticks per microsecond
            if (valid == 1) {
                double angle = ticksToDegrees(sweepDuration);
                if(rotor==0){
                    horizontal_angle[lighthouse].push_back(angle);
                    if(horizontal_angle[lighthouse].size()>values_in_plot)
                        horizontal_angle[lighthouse].pop_front();
                }else{
                    vertical_angle[lighthouse].push_back(angle);
                    if(vertical_angle[lighthouse].size()>values_in_plot)
                        vertical_angle[lighthouse].pop_front();
                }
                time[rotor+lighthouse*2].push_back(message_counter[rotor+lighthouse*2]);
                if(time[rotor+lighthouse*2].size()>values_in_plot)
                    time[rotor+lighthouse*2].pop_front();

                break;
            }
        }
        id++;
    }
    if((message_counter[rotor+lighthouse*2]++)%10==0)
        emit newData();
}

void RoboyDarkRoom::receiveStatistics(const roboy_communication_middleware::DarkRoomStatistics::ConstPtr &msg){
    ROS_DEBUG_THROTTLE(10, "receiving statistics data");
    for(uint i=0;i<msg->updateFrequency_horizontal.size();i++){
        updateFrequencies[msg->lighthouse][i][0].push_back(msg->updateFrequency_horizontal[i]);
        updateFrequencies[msg->lighthouse][i][1].push_back(msg->updateFrequency_vertical[i]);
        if(updateFrequencies[msg->lighthouse][i][0].size()>50)
            updateFrequencies[msg->lighthouse][i][0].pop_front();
        if(updateFrequencies[msg->lighthouse][i][1].size()>50)
            updateFrequencies[msg->lighthouse][i][1].pop_front();
    }
    statistics_time[msg->lighthouse].push_back(message_counter_statistics[msg->lighthouse]++);
    if(statistics_time[msg->lighthouse].size()>50)
        statistics_time[msg->lighthouse].pop_front();

    emit newStatisticsData();
}

void RoboyDarkRoom::plotData() {
    ui.horizontal_angle_lighthouse_1->graph(0)->setData(time[0], horizontal_angle[0]);
    ui.horizontal_angle_lighthouse_2->graph(0)->setData(time[3], horizontal_angle[1]);
    ui.vertical_angle_lighthouse_1->graph(0)->setData(time[1], vertical_angle[0]);
    ui.vertical_angle_lighthouse_2->graph(0)->setData(time[2], vertical_angle[1]);
    ui.horizontal_angle_lighthouse_1->xAxis->rescale();
    ui.horizontal_angle_lighthouse_2->xAxis->rescale();
    ui.vertical_angle_lighthouse_1->xAxis->rescale();
    ui.vertical_angle_lighthouse_2->xAxis->rescale();
    ui.horizontal_angle_lighthouse_1->replot();
    ui.horizontal_angle_lighthouse_2->replot();
    ui.vertical_angle_lighthouse_1->replot();
    ui.vertical_angle_lighthouse_2->replot();
}

void RoboyDarkRoom::plotStatisticsData(){
    for(auto &sensor:updateFrequencies[0]){
        ui.update_frequencies_horizontal_lighthouse_1->graph(sensor.first)->setData(
                statistics_time[0], updateFrequencies[0][sensor.first][0]);
        ui.update_frequencies_vertical_lighthouse_1->graph(sensor.first)->setData(
                statistics_time[0], updateFrequencies[0][sensor.first][1]);
        ui.update_frequencies_horizontal_lighthouse_2->graph(sensor.first)->setData(
                statistics_time[1], updateFrequencies[1][sensor.first][0]);
        ui.update_frequencies_vertical_lighthouse_2->graph(sensor.first)->setData(
                statistics_time[1], updateFrequencies[1][sensor.first][1]);
    }
    ui.update_frequencies_horizontal_lighthouse_1->xAxis->rescale();
    ui.update_frequencies_vertical_lighthouse_1->xAxis->rescale();
    ui.update_frequencies_horizontal_lighthouse_2->xAxis->rescale();
    ui.update_frequencies_vertical_lighthouse_2->xAxis->rescale();
    ui.update_frequencies_horizontal_lighthouse_1->replot();
    ui.update_frequencies_vertical_lighthouse_1->replot();
    ui.update_frequencies_horizontal_lighthouse_2->replot();
    ui.update_frequencies_vertical_lighthouse_2->replot();
}

PLUGINLIB_DECLARE_CLASS(roboy_darkroom, RoboyDarkRoom, RoboyDarkRoom, rqt_gui_cpp::Plugin)
