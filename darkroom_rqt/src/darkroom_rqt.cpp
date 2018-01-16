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
    clearAll();
    publish_transform = false;
    update_tracked_object_info = false;

    if (transform_thread->joinable()) {
        ROS_INFO("waiting for transform thread to shut down");
        transform_thread->join();
    }

    if (update_tracked_object_info_thread->joinable()) {
        ROS_INFO("waiting for update_tracked_object_info_thread to shut down");
        update_tracked_object_info_thread->join();
    }

    delete model;
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

    text["lighthouse_ID_1"] = widget_->findChild<QLineEdit *>("lighthouse_ID_1");
    text["lighthouse_acc_x_1"] = widget_->findChild<QLineEdit *>("lighthouse_acc_x_1");
    text["lighthouse_acc_y_1"] = widget_->findChild<QLineEdit *>("lighthouse_acc_y_1");
    text["lighthouse_acc_z_1"] = widget_->findChild<QLineEdit *>("lighthouse_acc_z_1");
    text["lighthouse_firmware_version_1"] = widget_->findChild<QLineEdit *>("lighthouse_firmware_version_1");
    text["lighthouse_hardware_version_1"] = widget_->findChild<QLineEdit *>("lighthouse_hardware_version_1");
    text["lighthouse_protocol_version_1"] = widget_->findChild<QLineEdit *>("lighthouse_protocol_version_1");
    text["lighthouse_selected_mode_1"] = widget_->findChild<QLineEdit *>("lighthouse_selected_mode_1");
    button["lighthouse_use_factory_calibration_data_phase_1"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_phase_1");
    button["lighthouse_use_factory_calibration_data_tilt_1"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_tilt_1");
    button["lighthouse_use_factory_calibration_data_gphase_1"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_gphase_1");
    button["lighthouse_use_factory_calibration_data_gmag_1"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_gmag_1");

    text["lighthouse_ID_2"] = widget_->findChild<QLineEdit *>("lighthouse_ID_2");
    text["lighthouse_acc_x_2"] = widget_->findChild<QLineEdit *>("lighthouse_acc_x_2");
    text["lighthouse_acc_y_2"] = widget_->findChild<QLineEdit *>("lighthouse_acc_y_2");
    text["lighthouse_acc_z_2"] = widget_->findChild<QLineEdit *>("lighthouse_acc_z_2");
    text["lighthouse_firmware_version_2"] = widget_->findChild<QLineEdit *>("lighthouse_firmware_version_2");
    text["lighthouse_hardware_version_2"] = widget_->findChild<QLineEdit *>("lighthouse_hardware_version_2");
    text["lighthouse_protocol_version_2"] = widget_->findChild<QLineEdit *>("lighthouse_protocol_version_2");
    text["lighthouse_selected_mode_2"] = widget_->findChild<QLineEdit *>("lighthouse_selected_mode_2");
    button["lighthouse_use_factory_calibration_data_phase_2"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_phase_2");
    button["lighthouse_use_factory_calibration_data_tilt_2"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_tilt_2");
    button["lighthouse_use_factory_calibration_data_gphase_2"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_gphase_2");
    button["lighthouse_use_factory_calibration_data_gmag_2"] = widget_->findChild<QPushButton *>(
            "lighthouse_use_factory_calibration_data_gmag_2");
    button["add_tracked_object"] = widget_->findChild<QPushButton *>("add_tracked_object");
    button["remove_tracked_object"] = widget_->findChild<QPushButton *>("remove_tracked_object");

    text["broadcast_ip"] = widget_->findChild<QLineEdit *>("broadcast_ip");
    text["broadcast_port"] = widget_->findChild<QLineEdit *>("broadcast_port");

    button["triangulate"] = widget_->findChild<QPushButton *>("triangulate");
    button["show_rays"] = widget_->findChild<QPushButton *>("show_rays");
    button["show_distances"] = widget_->findChild<QPushButton *>("show_distances");
    button["pose_correction_least_squares"] = widget_->findChild<QPushButton *>("pose_correction_least_squares");
    button["pose_correction_particle_filter"] = widget_->findChild<QPushButton *>("pose_correction_particle_filter");
    button["position_estimation_relativ_sensor_distances"] = widget_->findChild<QPushButton *>(
            "position_estimation_relativ_sensor_distances");
    button["pose_estimation_relativ_sensor_distances"] = widget_->findChild<QPushButton *>(
            "pose_estimation_relativ_sensor_distances");
    button["reset_lighthouse_poses"] = widget_->findChild<QPushButton *>("reset_lighthouse_poses");
    button["switch_lighthouses"] = widget_->findChild<QPushButton *>("switch_lighthouses");
    button["calibrate_relative_distances"] = widget_->findChild<QPushButton *>("calibrate_relative_distances");
    button["record"] = widget_->findChild<QPushButton *>("record");
    button["connect_roboy"] = widget_->findChild<QPushButton *>("connect_roboy");
    button["connect_object"] = widget_->findChild<QPushButton *>("connect_object");
    button["clear_all"] = widget_->findChild<QPushButton *>("clear_all");
    button["object_pose_estimation_least_squares"] = widget_->findChild<QPushButton *>(
            "object_pose_estimation_least_squares");
    button["estimate_factory_calibration_values"] = widget_->findChild<QPushButton *>(
            "estimate_factory_calibration_values");
    button["reset_factory_calibration_values"] = widget_->findChild<QPushButton *>(
            "reset_factory_calibration_values");
    button["reset_pose"] = widget_->findChild<QPushButton *>("reset_pose");

    text["lighthouse_phase_horizontal_1"] = widget_->findChild<QLineEdit *>("lighthouse_phase_horizontal_1");
    text["lighthouse_phase_vertical_1"] = widget_->findChild<QLineEdit *>("lighthouse_phase_vertical_1");
    text["lighthouse_tilt_horizontal_1"] = widget_->findChild<QLineEdit *>("lighthouse_tilt_horizontal_1");
    text["lighthouse_tilt_vertical_1"] = widget_->findChild<QLineEdit *>("lighthouse_tilt_vertical_1");
    text["lighthouse_curve_horizontal_1"] = widget_->findChild<QLineEdit *>("lighthouse_curve_horizontal_1");
    text["lighthouse_curve_vertical_1"] = widget_->findChild<QLineEdit *>("lighthouse_curve_vertical_1");
    text["lighthouse_gibphase_horizontal_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase_horizontal_1");
    text["lighthouse_gibphase_vertical_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase_vertical_1");
    text["lighthouse_gibmag_horizontal_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag_horizontal_1");
    text["lighthouse_gibmag_vertical_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag_vertical_1");

    text["lighthouse_phase_horizontal_2"] = widget_->findChild<QLineEdit *>("lighthouse_phase_horizontal_2");
    text["lighthouse_phase_vertical_2"] = widget_->findChild<QLineEdit *>("lighthouse_phase_vertical_2");
    text["lighthouse_tilt_horizontal_2"] = widget_->findChild<QLineEdit *>("lighthouse_tilt_horizontal_2");
    text["lighthouse_tilt_vertical_2"] = widget_->findChild<QLineEdit *>("lighthouse_tilt_vertical_2");
    text["lighthouse_curve_horizontal_2"] = widget_->findChild<QLineEdit *>("lighthouse_curve_horizontal_2");
    text["lighthouse_curve_vertical_2"] = widget_->findChild<QLineEdit *>("lighthouse_curve_vertical_2");
    text["lighthouse_gibphase_horizontal_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase_horizontal_2");
    text["lighthouse_gibphase_vertical_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase_vertical_2");
    text["lighthouse_gibmag_horizontal_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag_horizontal_2");
    text["lighthouse_gibmag_vertical_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag_vertical_2");

    slider["lighthouse_phase_horizontal_1"] = widget_->findChild<QSlider *>("lighthouse_phase_horizontal_slider_1");
    slider["lighthouse_phase_vertical_1"] = widget_->findChild<QSlider *>("lighthouse_phase_vertical_slider_1");
    slider["lighthouse_tilt_horizontal_1"] = widget_->findChild<QSlider *>("lighthouse_tilt_horizontal_slider_1");
    slider["lighthouse_tilt_vertical_1"] = widget_->findChild<QSlider *>("lighthouse_tilt_vertical_slider_1");
    slider["lighthouse_curve_horizontal_1"] = widget_->findChild<QSlider *>("lighthouse_curve_horizontal_slider_1");
    slider["lighthouse_curve_vertical_1"] = widget_->findChild<QSlider *>("lighthouse_curve_vertical_slider_1");
    slider["lighthouse_gibphase_horizontal_1"] = widget_->findChild<QSlider *>(
            "lighthouse_gibphase_horizontal_slider_1");
    slider["lighthouse_gibphase_vertical_1"] = widget_->findChild<QSlider *>("lighthouse_gibphase_vertical_slider_1");
    slider["lighthouse_gibmag_horizontal_1"] = widget_->findChild<QSlider *>("lighthouse_gibmag_horizontal_slider_1");
    slider["lighthouse_gibmag_vertical_1"] = widget_->findChild<QSlider *>("lighthouse_gibmag_vertical_slider_1");

    slider["lighthouse_phase_horizontal_2"] = widget_->findChild<QSlider *>("lighthouse_phase_horizontal_slider_2");
    slider["lighthouse_phase_vertical_2"] = widget_->findChild<QSlider *>("lighthouse_phase_vertical_slider_2");
    slider["lighthouse_tilt_horizontal_2"] = widget_->findChild<QSlider *>("lighthouse_tilt_horizontal_slider_2");
    slider["lighthouse_tilt_vertical_2"] = widget_->findChild<QSlider *>("lighthouse_tilt_vertical_slider_2");
    slider["lighthouse_curve_horizontal_2"] = widget_->findChild<QSlider *>("lighthouse_curve_horizontal_slider_2");
    slider["lighthouse_curve_vertical_2"] = widget_->findChild<QSlider *>("lighthouse_curve_vertical_slider_2");
    slider["lighthouse_gibphase_horizontal_2"] = widget_->findChild<QSlider *>(
            "lighthouse_gibphase_horizontal_slider_2");
    slider["lighthouse_gibphase_vertical_2"] = widget_->findChild<QSlider *>("lighthouse_gibphase_vertical_slider_2");
    slider["lighthouse_gibmag_horizontal_2"] = widget_->findChild<QSlider *>("lighthouse_gibmag_horizontal_slider_2");
    slider["lighthouse_gibmag_vertical_2"] = widget_->findChild<QSlider *>("lighthouse_gibmag_vertical_slider_2");

    for(auto &s:slider){
        QObject::connect(s.second, SIGNAL(valueChanged(int)), this, SLOT(updateCalibrationValues()));

    }
    updateCalibrationValues();

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
    button["pose_estimation_relativ_sensor_distances"]->setToolTip(
            "Use the known relative distances between sensors to estimate the distances to each lighthouse. Then estimate"
                    "a relative object pose");
    button["reset_lighthouse_poses"]->setToolTip("reset the lighthouse poses\nto the slider values");
    button["switch_lighthouses"]->setToolTip("switch lighthouses");
    button["calibrate_relative_distances"]->setToolTip("calibrate the relative distances\nof an unknown object "
                                                               "by averaging the triangulated\npositions for a couple of seconds."
                                                               "The result is written into a\nconfig file in calibrated_objects");
    button["record"]->setToolTip("records the sensor data to a log file");
    button["connect_roboy"]->setToolTip("subscribe to DarkRoom sensory data via ROS message");
    button["connect_object"]->setToolTip("subscribe to DarkRoom sensory data via UDP message");
    button["clear_all"]->setToolTip("clears all visualization markers");
    button["object_pose_estimation_least_squares"]->setToolTip("estimates object pose using least squares minimizer");
    button["add_tracked_object"]->setToolTip("adds the selected tracked object from the file browser");
    button["remove_tracked_object"]->setToolTip("removes the selected tracked object");

    ui.simulate->setToolTip("check this to simulate the sensor data");
    ui.random_pose->setToolTip("check this to randomly change the pose of all tracked objects");

    QObject::connect(button["triangulate"], SIGNAL(clicked()), this, SLOT(startTriangulation()));
    QObject::connect(button["show_rays"], SIGNAL(clicked()), this, SLOT(showRays()));
    QObject::connect(button["show_distances"], SIGNAL(clicked()), this, SLOT(showDistances()));
    QObject::connect(button["pose_correction_least_squares"], SIGNAL(clicked()), this,
                     SLOT(startPoseEstimationSensorCloud()));
    QObject::connect(button["position_estimation_relativ_sensor_distances"], SIGNAL(clicked()), this,
                     SLOT(startEstimateSensorPositionsUsingRelativeDistances()));
    QObject::connect(button["pose_estimation_relativ_sensor_distances"], SIGNAL(clicked()), this,
                     SLOT(startEstimateObjectPoseUsingRelativeDistances()));
    QObject::connect(button["reset_lighthouse_poses"], SIGNAL(clicked()), this, SLOT(resetLighthousePoses()));
    QObject::connect(button["calibrate_relative_distances"], SIGNAL(clicked()), this,
                     SLOT(startCalibrateRelativeSensorDistances()));
    QObject::connect(button["switch_lighthouses"], SIGNAL(clicked()), this, SLOT(switchLighthouses()));
    QObject::connect(button["record"], SIGNAL(clicked()), this, SLOT(record()));
    QObject::connect(button["connect_roboy"], SIGNAL(clicked()), this, SLOT(connectRoboy()));
    QObject::connect(button["connect_object"], SIGNAL(clicked()), this, SLOT(connectObject()));
    QObject::connect(button["clear_all"], SIGNAL(clicked()), this, SLOT(clearAll()));
    QObject::connect(button["object_pose_estimation_least_squares"], SIGNAL(clicked()), this,
                     SLOT(startObjectPoseEstimationSensorCloud()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_phase_1"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_tilt_1"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_gphase_1"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_gmag_1"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_phase_2"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_tilt_2"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_gphase_2"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["lighthouse_use_factory_calibration_data_gmag_2"], SIGNAL(clicked()), this,
                     SLOT(useFactoryCalibrationData()));
    QObject::connect(button["add_tracked_object"], SIGNAL(clicked()), this, SLOT(addTrackedObject()));
    QObject::connect(button["remove_tracked_object"], SIGNAL(clicked()), this, SLOT(removeTrackedObject()));
    QObject::connect(button["estimate_factory_calibration_values"], SIGNAL(clicked()), this, SLOT(estimateFactoryCalibration()));
    QObject::connect(button["reset_factory_calibration_values"], SIGNAL(clicked()), this, SLOT(resetFactoryCalibration()));
    QObject::connect(button["reset_pose"], SIGNAL(clicked()), this, SLOT(resetPose()));

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
    ootx_sub = nh->subscribe("/roboy/middleware/DarkRoom/ootx", 1, &RoboyDarkRoom::receiveOOTXData, this);
    aruco_pose_sub = nh->subscribe("/roboy/middleware/ArucoPose", 1, &RoboyDarkRoom::receiveArucoPose, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    resetLighthousePoses();

    publish_transform = true;
    transform_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyDarkRoom::transformPublisher, this));
    transform_thread->detach();

    update_tracked_object_info = true;
    update_tracked_object_info_thread = boost::shared_ptr<std::thread>(
            new std::thread(&RoboyDarkRoom::updateTrackedObjectInfo, this));
    update_tracked_object_info_thread->detach();

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

    for (uint i = 0; i < 32; i++) {
        QColor color(((float) rand() / RAND_MAX) * 255, ((float) rand() / RAND_MAX) * 255,
                     ((float) rand() / RAND_MAX) * 255);
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

    model = new QFileSystemModel;
    string package_path = ros::package::getPath("roboy_models");
    model->setRootPath(QString(package_path.c_str()));
    ui.tracked_object_browser->setModel(model);
    ui.tracked_object_browser->setRootIndex(model->index(QString(package_path.c_str())));

    QScrollArea *scrollArea = widget_->findChild<QScrollArea *>("tracked_objects");
    scrollArea->setBackgroundRole(QPalette::Window);
    scrollArea->setFrameShadow(QFrame::Plain);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);

    //vertical box that contains all the checkboxes for the filters
    QWidget *trackedObjects_scrollarea = new QWidget(widget_);
    trackedObjects_scrollarea->setObjectName("tracked_objects_scrollarea");
    trackedObjects_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    trackedObjects_scrollarea->setLayout(new QVBoxLayout(trackedObjects_scrollarea));
    scrollArea->setWidget(trackedObjects_scrollarea);
}

void RoboyDarkRoom::shutdownPlugin() {

}

void RoboyDarkRoom::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                 qt_gui_cpp::Settings &instance_settings) const {
}

void RoboyDarkRoom::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                    const qt_gui_cpp::Settings &instance_settings) {
}

void RoboyDarkRoom::connectRoboy() {
    ROS_DEBUG("connect roboy clicked");
    string package_path = ros::package::getPath("roboy_models");
    // vector<fs::path> roboy_parts = {
    //         package_path+"/Roboy2.0_Upper_Body_Xylophone_simplified/lighthouseSensors/xylophone.yaml",
    //         package_path+"/Roboy2.0_Upper_Body_Xylophone_simplified/lighthouseSensors/head.yaml",
    //         package_path+"/Roboy2.0_Upper_Body_Xylophone_simplified/lighthouseSensors/torso.yaml",
    //         package_path+"/Roboy2.0_Upper_Body_Xylophone_simplified/lighthouseSensors/upper_arm_left.yaml",
    //         package_path+"/Roboy2.0_Upper_Body_Xylophone_simplified/lighthouseSensors/upper_arm_right.yaml"
    // };
    vector<fs::path> roboy_parts = {
            package_path + "/TestCube/calibration.yaml"
//            package_path+"/Roboy2.0_Head_simplified/lighthouseSensors/upper_arm_left.yaml",
//            package_path+"/Roboy2.0_Head_simplified/lighthouseSensors/lower_arm_left.yaml",
//            package_path+"/Roboy2.0_Head_simplified/lighthouseSensors/upper_arm_right.yaml",
//            package_path+"/Roboy2.0_Head_simplified/lighthouseSensors/lower_arm_right.yaml"
    };
    for (auto &part:roboy_parts) {
        if (!fileExists(part.c_str())) {
            ROS_ERROR("could not connect Roboy, check the path %s", part.c_str());
            return;
        }
        if (!addTrackedObject(part.c_str()))
            continue;
    }
    if (ui.simulate->isChecked()) {
        pair<LighthouseSimulatorPtr, LighthouseSimulatorPtr> simulation;

        simulation.first.reset(new LighthouseSimulator(LIGHTHOUSE_A, roboy_parts));
        simulation.second.reset(new LighthouseSimulator(LIGHTHOUSE_B, roboy_parts));
        // lets start one imu publishing thread
        simulation.first->imu_thread.reset(new boost::thread([simulation]() { simulation.first->PublishImuData(); }));

        lighthouse_simulation.push_back(simulation);
    }
}

void RoboyDarkRoom::connectObject() {
    ROS_DEBUG("connect object clicked");
    if (!addTrackedObject())
        return;
    bool ok;
    trackedObjects.back()->connectObject(text["broadcast_ip"]->text().toStdString().c_str(),
                                         text["broadcast_port"]->text().toInt(&ok));
}

void RoboyDarkRoom::clearAll() {
    ROS_DEBUG("clear all clicked");
    for (auto const &object:trackedObjects) {
        object->clearAll();
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
    lighthouse1.setRotation(quat);
    lighthouse1.setOrigin(tf::Vector3(0, -2.849, -0.115));
//    lighthouse1.setOrigin(tf::Vector3(0, -2, 0));
    lighthouse2.setRotation(quat);
    lighthouse2.setOrigin(tf::Vector3(0, -2.849, -0.115));
//    lighthouse2.setOrigin(tf::Vector3(0, -2, 0));
}

void RoboyDarkRoom::record() {
    ROS_DEBUG("record clicked");
    if (trackedObjects.empty())
        button["record"]->setChecked(false);
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object->mux);
        object->record(button["record"]->isChecked());
    }
}

void RoboyDarkRoom::showRays() {
    ROS_DEBUG("show rays clicked");

    for (uint i = 0; i < trackedObjects.size(); i++) {
//        lock_guard<mutex>(trackedObjects[i]->mux);
        if (button["show_rays"]->isChecked()) {
            ROS_INFO("starting rays thread");
            trackedObjects[i]->rays = true;
            trackedObjects[i]->rays_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread(
                            [this, i]() { this->trackedObjects[i]->publishRays(); }
                    ));
            trackedObjects[i]->rays_thread->detach();
        } else {
            if (trackedObjects[i]->rays_thread != nullptr) {
                ROS_INFO("stopping rays thread");
                trackedObjects[i]->rays = false;
                if (trackedObjects[i]->rays_thread->joinable()) {
                    ROS_INFO_THROTTLE(1, "Waiting for rays thread to terminate");
                    trackedObjects[i]->rays_thread->join();
                }
            }
        }
    }
}

void RoboyDarkRoom::showDistances() {
    ROS_DEBUG("show distances clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object->mux);
        object->distances = object->rays = button["show_distances"]->isChecked();
    }
}

void RoboyDarkRoom::switchLighthouses() {
    ROS_DEBUG("switch lighthouses clicked");
    for (auto const &object:trackedObjects) {
        lock_guard<mutex>(object->mux);
        object->switchLighthouses(button["switch_lighthouses"]->isChecked());
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
//        lock_guard<mutex>(trackedObjects[i]->mux);
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
                    ROS_INFO_THROTTLE(1, "Waiting for tracking thread to terminate");
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
                    ROS_INFO_THROTTLE(1, "Waiting for pose estimation thread to terminate");
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

void RoboyDarkRoom::startEstimateObjectPoseUsingRelativeDistances() {
    ROS_DEBUG("pose_estimation_relativ_sensor_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        ROS_INFO("starting relativ pose thread");
        trackedObjects[i]->relative_pose_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateObjectPoseUsingRelativeDistances();
                }));
        trackedObjects[i]->relative_pose_thread->detach();
    }
}

void RoboyDarkRoom::transformPublisher() {
    ros::Rate rate(60);
    while (publish_transform) {
        lock_guard<mutex> lock(mux);
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
//        for (auto &simulated:lighthouse_simulation) {
//            tf_broadcaster.sendTransform(tf::StampedTransform(simulated.second->relative_object_pose, ros::Time::now(),
//                                                              (simulated.second->id == 0 ? "lighthouse1"
//                                                                                         : "lighthouse2"),
//                                                              simulated.second->name.c_str()));
//        }
//        if (ui.simulate->isChecked()){
            if (ui.random_pose->isChecked()) {
                // randomly moves all objects
                for (auto &object:trackedObjects) {
                    object->pose.setOrigin( tf::Vector3(0.2*sin(random_pose_x),0.2*sin(random_pose_y),0.2*sin(random_pose_z)));
                    tf::Quaternion q(0, 0, 0, 1);
                    q.setRPY(random_pose_roll, random_pose_pitch, random_pose_yaw);
                    object->pose.setRotation(q);
                    if(ui.random_pose_x->isChecked())
                        random_pose_x += 0.002;
                    if(ui.random_pose_y->isChecked())
                        random_pose_y += 0.002;
                    if(ui.random_pose_z->isChecked())
                        random_pose_z += 0.002;
                    if(ui.random_pose_roll->isChecked())
                        random_pose_roll += 0.002;
                    if(ui.random_pose_pitch->isChecked())
                        random_pose_pitch += 0.002;
                    if(ui.random_pose_yaw->isChecked())
                        random_pose_yaw += 0.002;
                }
            }
            for (auto &object:trackedObjects) {
                tf_broadcaster.sendTransform(tf::StampedTransform(object->pose, ros::Time::now(),
                                                                  "world", object->name.c_str()));
            }
//        }

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
    } else if (strcmp(msg.marker_name.c_str(), "lighthouse2") == 0) {
        lighthouse2.setOrigin(position);
        lighthouse2.setRotation(orientation);
    } else if (strcmp(msg.marker_name.c_str(), "trackedObject") == 0) {
        for (auto &object:trackedObjects) {
            object->pose.setOrigin(position);
            object->pose.setRotation(orientation);
        }
    }
}

void RoboyDarkRoom::receiveSensorData(const roboy_communication_middleware::DarkRoom::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(10, "receiving sensor data");
    uint id = 0;
    uint lighthouse, rotor, sweepDuration;
    for (uint32_t const &data:msg->sensor_value) {
        if (ui.sensor_selector->value() == id) {
            lighthouse = (data >> 31) & 0x1;
            rotor = (data >> 30) & 0x1;
            int valid = (data >> 29) & 0x1;
            sweepDuration = (data & 0x1fffffff); // raw sensor duration is 50 ticks per microsecond
            if (valid == 1) {
                double angle = ticksToDegrees(sweepDuration);
                if (rotor == 0) {
                    horizontal_angle[lighthouse].push_back(angle);
                    if (horizontal_angle[lighthouse].size() > values_in_plot)
                        horizontal_angle[lighthouse].pop_front();
                } else {
                    vertical_angle[lighthouse].push_back(angle);
                    if (vertical_angle[lighthouse].size() > values_in_plot)
                        vertical_angle[lighthouse].pop_front();
                }
                time[rotor + lighthouse * 2].push_back(message_counter[rotor + lighthouse * 2]);
                if (time[rotor + lighthouse * 2].size() > values_in_plot)
                    time[rotor + lighthouse * 2].pop_front();

                break;
            }
        }
        id++;
    }
    if ((message_counter[rotor + lighthouse * 2]++) % 10 == 0 && ui.tabWidget->currentIndex() == 1)
            emit newData();
}

void RoboyDarkRoom::receiveStatistics(const roboy_communication_middleware::DarkRoomStatistics::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(10, "receiving statistics data");
    for (uint i = 0; i < msg->updateFrequency_horizontal.size(); i++) {
        updateFrequencies[msg->lighthouse][i][0].push_back(msg->updateFrequency_horizontal[i]);
        updateFrequencies[msg->lighthouse][i][1].push_back(msg->updateFrequency_vertical[i]);
        if (updateFrequencies[msg->lighthouse][i][0].size() > 50)
            updateFrequencies[msg->lighthouse][i][0].pop_front();
        if (updateFrequencies[msg->lighthouse][i][1].size() > 50)
            updateFrequencies[msg->lighthouse][i][1].pop_front();
    }
    statistics_time[msg->lighthouse].push_back(message_counter_statistics[msg->lighthouse]++);
    if (statistics_time[msg->lighthouse].size() > 50)
        statistics_time[msg->lighthouse].pop_front();

    if (ui.tabWidget->currentIndex() == 2)
            emit newStatisticsData();
}

void RoboyDarkRoom::receiveOOTXData(const roboy_communication_middleware::DarkRoomOOTX::ConstPtr &msg) {
    if (msg->lighthouse == 0) {
        text["lighthouse_firmware_version_1"]->setText(QString::number(msg->fw_version >> 6 & 0x3FF, 16));
        text["lighthouse_protocol_version_1"]->setText(QString::number(msg->fw_version & 0x1F, 10));
        text["lighthouse_ID_1"]->setText(QString::number(msg->ID, 16));
        text["lighthouse_hardware_version_1"]->setText(QString::number(msg->hw_version, 16));
        text["lighthouse_selected_mode_1"]->setText(QString::number(msg->mode));
        text["lighthouse_acc_x_1"]->setText(QString::number(msg->accel_dir_x));
        text["lighthouse_acc_y_1"]->setText(QString::number(msg->accel_dir_y));
        text["lighthouse_acc_z_1"]->setText(QString::number(msg->accel_dir_z));
        button["lighthouse_use_factory_calibration_data_phase_1"]->setEnabled(true);
        button["lighthouse_use_factory_calibration_data_tilt_1"]->setEnabled(true);
        button["lighthouse_use_factory_calibration_data_gphase_1"]->setEnabled(true);
        button["lighthouse_use_factory_calibration_data_gmag_1"]->setEnabled(true);
    } else {
        text["lighthouse_firmware_version_2"]->setText(QString::number(msg->fw_version >> 6 & 0x3FF, 16));
        text["lighthouse_protocol_version_2"]->setText(QString::number(msg->fw_version & 0x1F, 10));
        text["lighthouse_ID_2"]->setText(QString::number(msg->ID, 16));
        text["lighthouse_hardware_version_2"]->setText(QString::number(msg->hw_version, 16));
        text["lighthouse_selected_mode_2"]->setText(QString::number(msg->mode));
        text["lighthouse_acc_x_2"]->setText(QString::number(msg->accel_dir_x));
        text["lighthouse_acc_y_2"]->setText(QString::number(msg->accel_dir_y));
        text["lighthouse_acc_z_2"]->setText(QString::number(msg->accel_dir_z));
        button["lighthouse_use_factory_calibration_data_phase_2"]->setEnabled(true);
        button["lighthouse_use_factory_calibration_data_tilt_2"]->setEnabled(true);
        button["lighthouse_use_factory_calibration_data_gphase_2"]->setEnabled(true);
        button["lighthouse_use_factory_calibration_data_gmag_2"]->setEnabled(true);
    }
}

bool RoboyDarkRoom::fileExists(const string &filepath) {
    struct stat buffer;
    return (stat(filepath.c_str(), &buffer) == 0);
}

void RoboyDarkRoom::updateTrackedObjectInfo() {
    ros::Rate rate(10);
    while (update_tracked_object_info) {
        {
            lock_guard<mutex> lock(mux);
            for (int i = 0; i < trackedObjects.size(); i++) {
                char str[100];
                sprintf(str, "%d/%d", trackedObjects[i]->active_sensors, (int) trackedObjects[i]->sensors.size());
                trackedObjectsInfo[i].activeSensors->setText(str);
            }
        }
        rate.sleep();
    }
}

void RoboyDarkRoom::receiveArucoPose(const roboy_communication_middleware::ArucoPose::ConstPtr &msg){
    int i=0;
    // running mean and variance (cf https://www.johndcook.com/blog/standard_deviation/ )
    stringstream str;
    for(int id:msg->id){
        if(receive_counter.find(id)==receive_counter.end()){
            receive_counter[id] = 1;
            aruco_position_mean[id].setZero();
            aruco_position_variance[id].setZero();
        }

        Vector3d pos(msg->pose[i].position.x,
                     msg->pose[i].position.y,
                     msg->pose[i].position.z);
        // using lazy average at this point
        Vector3d new_mean = aruco_position_mean[id]*0.9 + pos * 0.1;
        aruco_position_variance[id](0) += (pos(0)-aruco_position_mean[id](0))*(pos(0)-new_mean(0));
        aruco_position_variance[id](1) += (pos(1)-aruco_position_mean[id](1))*(pos(1)-new_mean(1));
        aruco_position_variance[id](2) += (pos(2)-aruco_position_mean[id](2))*(pos(2)-new_mean(2));
        aruco_position_mean[id] = new_mean;
        str << "\naruco id " << id << " \nmean " << aruco_position_mean[id].transpose() << " \nvariance " << aruco_position_variance[id].transpose() << endl;
        i++;
        receive_counter[id]++;
    }
    if(!ui.random_pose->isChecked()){
        for(auto &object:trackedObjects){
            object->pose.setOrigin(tf::Vector3(aruco_position_mean[282](0),aruco_position_mean[282](1),aruco_position_mean[282](2)));
        }
    }

    ROS_INFO_STREAM_THROTTLE(1,str.str());
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

void RoboyDarkRoom::plotStatisticsData() {
    for (auto &sensor:updateFrequencies[0]) {
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

void RoboyDarkRoom::useFactoryCalibrationData() {
    ROS_DEBUG("lighthouse_use_factory_calibration_data clicked");
    // phase
    if (button["lighthouse_use_factory_calibration_data_phase_1"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_phase[0] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_phase[0] = false;
        }
    }
    if (button["lighthouse_use_factory_calibration_data_phase_2"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_phase[1] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_phase[1] = false;
        }
    }
    // tilt
    if (button["lighthouse_use_factory_calibration_data_tilt_1"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_tilt[0] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_tilt[0] = false;
        }
    }
    if (button["lighthouse_use_factory_calibration_data_tilt_2"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_tilt[1] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_tilt[1] = false;
        }
    }
    // gibbous phase
    if (button["lighthouse_use_factory_calibration_data_gphase_1"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibphase[0] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibphase[0] = false;
        }
    }
    if (button["lighthouse_use_factory_calibration_data_gphase_2"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibphase[1] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibphase[1] = false;
        }
    }
    // gibbous magnitude
    if (button["lighthouse_use_factory_calibration_data_gmag_1"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibmag[0] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibmag[0] = false;
        }
    }
    if (button["lighthouse_use_factory_calibration_data_gmag_2"]->isChecked()) {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibmag[1] = true;
        }
    } else {
        for (uint i = 0; i < trackedObjects.size(); i++) {
            lock_guard<mutex>(trackedObjects[i]->mux);
            trackedObjects[i]->use_lighthouse_calibration_data_gibmag[1] = false;
        }
    }
}

bool RoboyDarkRoom::addTrackedObject(const char *config_file_path) {
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    if (strlen(config_file_path) == 0) {
        QModelIndexList indexList = ui.tracked_object_browser->selectionModel()->selectedIndexes();
        if (indexList.empty())
            return false;
        if (!newObject->init(model->filePath(indexList[0]).toStdString().c_str()))
            return false;
    } else {
        if (!newObject->init(config_file_path))
            return false;
    }

    ROS_DEBUG_STREAM("adding tracked object " << config_file_path);
    trackedObjects.push_back(newObject);
    object_counter++;


    TrackedObjectInfo info;

    QWidget *tracked_objects_scrollarea = widget_->findChild<QWidget *>("tracked_objects_scrollarea");
    info.widget = new QWidget(tracked_objects_scrollarea);
    char str[100];
    sprintf(str, "%s", newObject->name.c_str());
    info.widget->setObjectName(str);
    info.widget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    info.widget->setLayout(new QHBoxLayout(info.widget));

    QCheckBox *box = new QCheckBox;
    box->setChecked(true);
    info.widget->layout()->addWidget(box);

    QLabel *name = new QLabel(info.widget);
    name->setFixedSize(150, 30);
    name->setText(str);
    info.widget->layout()->addWidget(name);

    QLabel *activeSensors = new QLabel(info.widget);
    activeSensors->setFixedSize(50, 30);
    activeSensors->setText("active sensors");
    info.widget->layout()->addWidget(activeSensors);

    info.name = name;
    info.activeSensors = activeSensors;
    info.selected = box;

    trackedObjectsInfo.push_back(info);

    tracked_objects_scrollarea->layout()->addWidget(info.widget);
    return true;
}

/**
 * removes the selected tracked object
 */
void RoboyDarkRoom::removeTrackedObject() {
    int i = 0;
    for (auto it = trackedObjects.begin(); it != trackedObjects.end();) {
        if (trackedObjectsInfo[i].selected->isChecked()) {
            // remove gui elements
            delete trackedObjectsInfo[i].name;
            delete trackedObjectsInfo[i].activeSensors;
            delete trackedObjectsInfo[i].selected;
            delete trackedObjectsInfo[i].widget;
            trackedObjectsInfo.erase(trackedObjectsInfo.begin() + i);
            // if we are simulating, shutdown lighthouse simulators
            if (ui.simulate->isChecked()) {
                if(!lighthouse_simulation.empty())
                    lighthouse_simulation.erase(lighthouse_simulation.begin() + i);
            }
            // erase the trackedObject
            it = trackedObjects.erase(it);
        } else {
            i++;
            ++it;
        }
    }
}

void RoboyDarkRoom::updateCalibrationValues(){
    text["lighthouse_phase_horizontal_1"]->setText(QString::number((slider["lighthouse_phase_horizontal_1"]->value()-50)/50.0/3.0));
    text["lighthouse_phase_vertical_1"]->setText(QString::number((slider["lighthouse_phase_vertical_1"]->value()-50)/50.0/3.0));
    text["lighthouse_tilt_horizontal_1"]->setText(QString::number((slider["lighthouse_tilt_horizontal_1"]->value()-50)/50.0/3.0));
    text["lighthouse_tilt_vertical_1"]->setText(QString::number((slider["lighthouse_tilt_vertical_1"]->value()-50)/50.0/3.0));
    text["lighthouse_curve_horizontal_1"]->setText(QString::number((slider["lighthouse_curve_horizontal_1"]->value()-50)/50.0/3.0));
    text["lighthouse_curve_vertical_1"]->setText(QString::number((slider["lighthouse_curve_vertical_1"]->value()-50)/50.0/3.0));
    text["lighthouse_gibphase_horizontal_1"]->setText(QString::number((slider["lighthouse_gibphase_horizontal_1"]->value()-50)/50.0/3.0));
    text["lighthouse_gibphase_vertical_1"]->setText(QString::number((slider["lighthouse_gibphase_vertical_1"]->value()-50)/50.0/3.0));
    text["lighthouse_gibmag_horizontal_1"]->setText(QString::number((slider["lighthouse_gibmag_horizontal_1"]->value()-50)/50.0/3.0));
    text["lighthouse_gibmag_vertical_1"]->setText(QString::number((slider["lighthouse_gibmag_vertical_1"]->value()-50)/50.0/3.0));

    text["lighthouse_phase_horizontal_2"]->setText(QString::number((slider["lighthouse_phase_horizontal_2"]->value()-50)/50.0/3.0));
    text["lighthouse_phase_vertical_2"]->setText(QString::number((slider["lighthouse_phase_vertical_2"]->value()-50)/50.0/3.0));
    text["lighthouse_tilt_horizontal_2"]->setText(QString::number((slider["lighthouse_tilt_horizontal_2"]->value()-50)/50.0/3.0));
    text["lighthouse_tilt_vertical_2"]->setText(QString::number((slider["lighthouse_tilt_vertical_2"]->value()-50)/50.0/3.0));
    text["lighthouse_curve_horizontal_2"]->setText(QString::number((slider["lighthouse_curve_horizontal_2"]->value()-50)/50.0/3.0));
    text["lighthouse_curve_vertical_2"]->setText(QString::number((slider["lighthouse_curve_vertical_2"]->value()-50)/50.0/3.0));
    text["lighthouse_gibphase_horizontal_2"]->setText(QString::number((slider["lighthouse_gibphase_horizontal_2"]->value()-50)/50.0/3.0));
    text["lighthouse_gibphase_vertical_2"]->setText(QString::number((slider["lighthouse_gibphase_vertical_2"]->value()-50)/50.0/3.0));
    text["lighthouse_gibmag_horizontal_2"]->setText(QString::number((slider["lighthouse_gibmag_horizontal_2"]->value()-50)/50.0/3.0));
    text["lighthouse_gibmag_vertical_2"]->setText(QString::number((slider["lighthouse_gibmag_vertical_2"]->value()-50)/50.0/3.0));

    // if there is no simulation we update the trackedObjects calibration values
    if(lighthouse_simulation.empty()){
        for(auto &object:trackedObjects){
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].phase = text["lighthouse_phase_horizontal_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][VERTICAL].phase = text["lighthouse_phase_vertical_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].tilt = text["lighthouse_tilt_horizontal_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][VERTICAL].tilt = text["lighthouse_tilt_vertical_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].curve = text["lighthouse_curve_horizontal_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][VERTICAL].curve = text["lighthouse_curve_vertical_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase = text["lighthouse_gibphase_horizontal_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][VERTICAL].gibphase = text["lighthouse_gibphase_vertical_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag = text["lighthouse_gibmag_horizontal_1"]->text().toDouble();
            object->calibration[LIGHTHOUSE_A][VERTICAL].gibmag = text["lighthouse_gibmag_vertical_1"]->text().toDouble();

            object->calibration[LIGHTHOUSE_B][HORIZONTAL].phase = text["lighthouse_phase_horizontal_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][VERTICAL].phase = text["lighthouse_phase_vertical_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].tilt = text["lighthouse_tilt_horizontal_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][VERTICAL].tilt = text["lighthouse_tilt_vertical_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].curve = text["lighthouse_curve_horizontal_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][VERTICAL].curve = text["lighthouse_curve_vertical_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase = text["lighthouse_gibphase_horizontal_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][VERTICAL].gibphase = text["lighthouse_gibphase_vertical_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag = text["lighthouse_gibmag_horizontal_2"]->text().toDouble();
            object->calibration[LIGHTHOUSE_B][VERTICAL].gibmag = text["lighthouse_gibmag_vertical_2"]->text().toDouble();
        }
    }else{ // we want to estimate the simulated calibration values
        for(auto &lighthouse:lighthouse_simulation){
            lighthouse.first->calibration[LIGHTHOUSE_A][HORIZONTAL].phase = text["lighthouse_phase_horizontal_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][VERTICAL].phase = text["lighthouse_phase_vertical_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][HORIZONTAL].tilt = text["lighthouse_tilt_horizontal_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][VERTICAL].tilt = text["lighthouse_tilt_vertical_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][HORIZONTAL].curve = text["lighthouse_curve_horizontal_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][VERTICAL].curve = text["lighthouse_curve_vertical_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase = text["lighthouse_gibphase_horizontal_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][VERTICAL].gibphase = text["lighthouse_gibphase_vertical_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag = text["lighthouse_gibmag_horizontal_1"]->text().toDouble();
            lighthouse.first->calibration[LIGHTHOUSE_A][VERTICAL].gibmag = text["lighthouse_gibmag_vertical_1"]->text().toDouble();

            lighthouse.second->calibration[LIGHTHOUSE_B][HORIZONTAL].phase = text["lighthouse_phase_horizontal_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][VERTICAL].phase = text["lighthouse_phase_vertical_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][HORIZONTAL].tilt = text["lighthouse_tilt_horizontal_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][VERTICAL].tilt = text["lighthouse_tilt_vertical_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][HORIZONTAL].curve = text["lighthouse_curve_horizontal_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][VERTICAL].curve = text["lighthouse_curve_vertical_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase = text["lighthouse_gibphase_horizontal_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][VERTICAL].gibphase = text["lighthouse_gibphase_vertical_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag = text["lighthouse_gibmag_horizontal_2"]->text().toDouble();
            lighthouse.second->calibration[LIGHTHOUSE_B][VERTICAL].gibmag = text["lighthouse_gibmag_vertical_2"]->text().toDouble();
        }
    }
}

void RoboyDarkRoom::estimateFactoryCalibration(){
    ROS_DEBUG("estimate_factory_calibration clicked");
    string package_path = ros::package::getPath("darkroom");
    string calibration_object_path = package_path + "/calibrated_objects/calibration.yaml";
    if (!fileExists(calibration_object_path.c_str())) {
        ROS_ERROR("could not load calibration object, check the path %s", calibration_object_path.c_str());
        return;
    }

    if (ui.simulate->isChecked()) {
        pair<LighthouseSimulatorPtr, LighthouseSimulatorPtr> simulation;

        vector<fs::path> parts = {calibration_object_path};

        simulation.first.reset(new LighthouseSimulator(LIGHTHOUSE_A, parts));
        simulation.second.reset(new LighthouseSimulator(LIGHTHOUSE_B, parts));

        simulation.first->calibration[LIGHTHOUSE_A][HORIZONTAL].phase = text["lighthouse_phase_horizontal_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][VERTICAL].phase = text["lighthouse_phase_vertical_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][HORIZONTAL].tilt = text["lighthouse_tilt_horizontal_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][VERTICAL].tilt = text["lighthouse_tilt_vertical_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][HORIZONTAL].curve = text["lighthouse_curve_horizontal_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][VERTICAL].curve = text["lighthouse_curve_vertical_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase = text["lighthouse_gibphase_horizontal_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][VERTICAL].gibphase = text["lighthouse_gibphase_vertical_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag = text["lighthouse_gibmag_horizontal_1"]->text().toDouble();
        simulation.first->calibration[LIGHTHOUSE_A][VERTICAL].gibmag = text["lighthouse_gibmag_vertical_1"]->text().toDouble();

        simulation.second->calibration[LIGHTHOUSE_B][HORIZONTAL].phase = text["lighthouse_phase_horizontal_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][VERTICAL].phase = text["lighthouse_phase_vertical_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][HORIZONTAL].tilt = text["lighthouse_tilt_horizontal_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][VERTICAL].tilt = text["lighthouse_tilt_vertical_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][HORIZONTAL].curve = text["lighthouse_curve_horizontal_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][VERTICAL].curve = text["lighthouse_curve_vertical_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase = text["lighthouse_gibphase_horizontal_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][VERTICAL].gibphase = text["lighthouse_gibphase_vertical_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag = text["lighthouse_gibmag_horizontal_2"]->text().toDouble();
        simulation.second->calibration[LIGHTHOUSE_B][VERTICAL].gibmag = text["lighthouse_gibmag_vertical_2"]->text().toDouble();

        lighthouse_simulation.push_back(simulation);
    }

    addTrackedObject(calibration_object_path.c_str());

    TrackedObjectPtr calibration_object = trackedObjects.back();

    calibration_object->pose.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion q;
    q.setRPY(0,0,0);
    calibration_object->pose.setRotation(q);

    ros::Duration d(2);
    // wait a bit to be sure there is sensor data available
    d.sleep();

    if(ui.estimate_lighthouse_1->isChecked()){
        calibration_object->calibration[LIGHTHOUSE_A][VERTICAL].reset();
        calibration_object->calibration[LIGHTHOUSE_A][HORIZONTAL].reset();
        if(calibration_object->estimateFactoryCalibration(LIGHTHOUSE_A)){
            text["lighthouse_phase_horizontal_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][HORIZONTAL].phase));
            text["lighthouse_phase_vertical_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][VERTICAL].phase));
            text["lighthouse_tilt_horizontal_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][HORIZONTAL].tilt));
            text["lighthouse_tilt_vertical_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][VERTICAL].tilt));
            text["lighthouse_curve_horizontal_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][HORIZONTAL].curve));
            text["lighthouse_curve_vertical_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][VERTICAL].curve));
            text["lighthouse_gibphase_horizontal_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase));
            text["lighthouse_gibphase_vertical_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][VERTICAL].gibphase));
            text["lighthouse_gibmag_horizontal_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag));
            text["lighthouse_gibmag_vertical_1"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_A][VERTICAL].gibmag));
        }
    }

    if(ui.estimate_lighthouse_2->isChecked()){
        calibration_object->calibration[LIGHTHOUSE_B][VERTICAL].reset();
        calibration_object->calibration[LIGHTHOUSE_B][HORIZONTAL].reset();
        if(calibration_object->estimateFactoryCalibration(LIGHTHOUSE_B)){
            text["lighthouse_phase_horizontal_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][HORIZONTAL].phase));
            text["lighthouse_phase_vertical_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][VERTICAL].phase));
            text["lighthouse_tilt_horizontal_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][HORIZONTAL].tilt));
            text["lighthouse_tilt_vertical_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][VERTICAL].tilt));
            text["lighthouse_curve_horizontal_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][HORIZONTAL].curve));
            text["lighthouse_curve_vertical_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][VERTICAL].curve));
            text["lighthouse_gibphase_horizontal_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase));
            text["lighthouse_gibphase_vertical_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][VERTICAL].gibphase));
            text["lighthouse_gibmag_horizontal_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag));
            text["lighthouse_gibmag_vertical_2"]->setText(QString::number(calibration_object->calibration[LIGHTHOUSE_B][VERTICAL].gibmag));
        }
    }

}

void RoboyDarkRoom::resetFactoryCalibration(){
    ROS_DEBUG("reset_factory_calibration_values clicked");
    text["lighthouse_phase_horizontal_1"]->setText(QString::number(0));
    text["lighthouse_phase_vertical_1"]->setText(QString::number(0));
    text["lighthouse_tilt_horizontal_1"]->setText(QString::number(0));
    text["lighthouse_tilt_vertical_1"]->setText(QString::number(0));
    text["lighthouse_curve_horizontal_1"]->setText(QString::number(0));
    text["lighthouse_curve_vertical_1"]->setText(QString::number(0));
    text["lighthouse_gibphase_horizontal_1"]->setText(QString::number(0));
    text["lighthouse_gibphase_vertical_1"]->setText(QString::number(0));
    text["lighthouse_gibmag_horizontal_1"]->setText(QString::number(0));
    text["lighthouse_gibmag_vertical_1"]->setText(QString::number(0));

    text["lighthouse_phase_horizontal_2"]->setText(QString::number(0));
    text["lighthouse_phase_vertical_2"]->setText(QString::number(0));
    text["lighthouse_tilt_horizontal_2"]->setText(QString::number(0));
    text["lighthouse_tilt_vertical_2"]->setText(QString::number(0));
    text["lighthouse_curve_horizontal_2"]->setText(QString::number(0));
    text["lighthouse_curve_vertical_2"]->setText(QString::number(0));
    text["lighthouse_gibphase_horizontal_2"]->setText(QString::number(0));
    text["lighthouse_gibphase_vertical_2"]->setText(QString::number(0));
    text["lighthouse_gibmag_horizontal_2"]->setText(QString::number(0));
    text["lighthouse_gibmag_vertical_2"]->setText(QString::number(0));
    for(auto &object:trackedObjects){
        object->calibration[LIGHTHOUSE_A][HORIZONTAL].reset();
        object->calibration[LIGHTHOUSE_A][VERTICAL].reset();
        object->calibration[LIGHTHOUSE_B][HORIZONTAL].reset();
        object->calibration[LIGHTHOUSE_B][VERTICAL].reset();
    }
}

void RoboyDarkRoom::resetPose(){
    for(auto &object:trackedObjects){
        object->pose.setOrigin(tf::Vector3(0,0,0));
        tf::Quaternion q(0, 0, 0, 1);
        q.setRPY(0, 0, 0);
        object->pose.setRotation(q);
        random_pose_x = 0;
        random_pose_y = 0;
        random_pose_z = 0;
        random_pose_roll = 0;
        random_pose_pitch = 0;
        random_pose_yaw = 0;
    }
}

PLUGINLIB_DECLARE_CLASS(roboy_darkroom, RoboyDarkRoom, RoboyDarkRoom, rqt_gui_cpp::Plugin)
