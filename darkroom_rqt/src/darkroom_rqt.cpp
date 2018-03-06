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
    mux.lock();
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
    mux.unlock();
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
    text["lighthouse_fault_detect_flags_1"] = widget_->findChild<QLineEdit *>("lighthouse_fault_detect_flags_1");
    text["lighthouse_desync_counter_1"] = widget_->findChild<QLineEdit *>("lighthouse_desync_counter_1");
    text["lighthouse_phase0_1"] = widget_->findChild<QLineEdit *>("lighthouse_phase0_1");
    text["lighthouse_phase1_1"] = widget_->findChild<QLineEdit *>("lighthouse_phase1_1");
    text["lighthouse_tilt0_1"] = widget_->findChild<QLineEdit *>("lighthouse_tilt0_1");
    text["lighthouse_tilt1_1"] = widget_->findChild<QLineEdit *>("lighthouse_tilt1_1");
    text["lighthouse_curve0_1"] = widget_->findChild<QLineEdit *>("lighthouse_curve0_1");
    text["lighthouse_curve1_1"] = widget_->findChild<QLineEdit *>("lighthouse_curve1_1");
    text["lighthouse_gibphase0_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase0_1");
    text["lighthouse_gibphase1_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase1_1");
    text["lighthouse_gibmag0_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag0_1");
    text["lighthouse_gibmag1_1"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag1_1");
    button["lighthouse_use_phase_1"] = widget_->findChild<QPushButton *>("lighthouse_use_phase_1");
    button["lighthouse_use_tilt_1"] = widget_->findChild<QPushButton *>("lighthouse_use_tilt_1");
    button["lighthouse_use_curve_1"] = widget_->findChild<QPushButton *>("lighthouse_use_curve_1");
    button["lighthouse_use_gibbous_1"] = widget_->findChild<QPushButton *>("lighthouse_use_gibbous_1");

    text["lighthouse_ID_2"] = widget_->findChild<QLineEdit *>("lighthouse_ID_2");
    text["lighthouse_acc_x_2"] = widget_->findChild<QLineEdit *>("lighthouse_acc_x_2");
    text["lighthouse_acc_y_2"] = widget_->findChild<QLineEdit *>("lighthouse_acc_y_2");
    text["lighthouse_acc_z_2"] = widget_->findChild<QLineEdit *>("lighthouse_acc_z_2");
    text["lighthouse_firmware_version_2"] = widget_->findChild<QLineEdit *>("lighthouse_firmware_version_2");
    text["lighthouse_hardware_version_2"] = widget_->findChild<QLineEdit *>("lighthouse_hardware_version_2");
    text["lighthouse_protocol_version_2"] = widget_->findChild<QLineEdit *>("lighthouse_protocol_version_2");
    text["lighthouse_selected_mode_2"] = widget_->findChild<QLineEdit *>("lighthouse_selected_mode_2");
    text["lighthouse_fault_detect_flags_2"] = widget_->findChild<QLineEdit *>("lighthouse_fault_detect_flags_2");
    text["lighthouse_desync_counter_2"] = widget_->findChild<QLineEdit *>("lighthouse_desync_counter_2");
    text["lighthouse_phase0_2"] = widget_->findChild<QLineEdit *>("lighthouse_phase0_2");
    text["lighthouse_phase1_2"] = widget_->findChild<QLineEdit *>("lighthouse_phase1_2");
    text["lighthouse_tilt0_2"] = widget_->findChild<QLineEdit *>("lighthouse_tilt0_2");
    text["lighthouse_tilt1_2"] = widget_->findChild<QLineEdit *>("lighthouse_tilt1_2");
    text["lighthouse_curve0_2"] = widget_->findChild<QLineEdit *>("lighthouse_curve0_2");
    text["lighthouse_curve1_2"] = widget_->findChild<QLineEdit *>("lighthouse_curve1_2");
    text["lighthouse_gibphase0_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase0_2");
    text["lighthouse_gibphase1_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibphase1_2");
    text["lighthouse_gibmag0_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag0_2");
    text["lighthouse_gibmag1_2"] = widget_->findChild<QLineEdit *>("lighthouse_gibmag1_2");
    button["lighthouse_use_phase_2"] = widget_->findChild<QPushButton *>("lighthouse_use_phase_2");
    button["lighthouse_use_tilt_2"] = widget_->findChild<QPushButton *>("lighthouse_use_tilt_2");
    button["lighthouse_use_curve_2"] = widget_->findChild<QPushButton *>("lighthouse_use_curve_2");
    button["lighthouse_use_gibbous_2"] = widget_->findChild<QPushButton *>("lighthouse_use_gibbous_2");
    button["add_tracked_object"] = widget_->findChild<QPushButton *>("add_tracked_object");
    button["remove_tracked_object"] = widget_->findChild<QPushButton *>("remove_tracked_object");

    text["broadcast_ip"] = widget_->findChild<QLineEdit *>("broadcast_ip");
    text["broadcast_port"] = widget_->findChild<QLineEdit *>("broadcast_port");

    button["triangulate"] = widget_->findChild<QPushButton *>("triangulate");
    button["show_rays"] = widget_->findChild<QPushButton *>("show_rays");
    button["show_distances"] = widget_->findChild<QPushButton *>("show_distances");
    button["pose_correction_least_squares"] = widget_->findChild<QPushButton *>("pose_correction_least_squares");
    button["position_estimation_relativ_sensor_distances"] = widget_->findChild<QPushButton *>(
            "position_estimation_relativ_sensor_distances");
    button["pose_estimation_relativ_sensor_distances"] = widget_->findChild<QPushButton *>(
            "pose_estimation_relativ_sensor_distances");
    button["pose_estimation_epnp"] = widget_->findChild<QPushButton *>(  "pose_estimation_epnp");
    button["pose_estimation_multi_lighthouse"] = widget_->findChild<QPushButton *>(  "pose_estimation_multi_lighthouse");
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
    button["estimate_factory_calibration_values_2"] = widget_->findChild<QPushButton *>(
            "estimate_factory_calibration_values_2");
    button["estimate_factory_calibration_values_epnp"] = widget_->findChild<QPushButton *>(
            "estimate_factory_calibration_values_epnp");
    button["estimate_factory_calibration_values_multi"] = widget_->findChild<QPushButton *>(
            "estimate_factory_calibration_values_multi");
    button["reset_factory_calibration_values"] = widget_->findChild<QPushButton *>(
            "reset_factory_calibration_values");
    button["use_steamVR_lighthouse_poses"] = widget_->findChild<QPushButton *>("use_steamVR_lighthouse_poses");
    button["compare_to_steamVR"] = widget_->findChild<QPushButton *>("compare_to_steamVR");
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
    button["pose_estimation_epnp"]->setToolTip(
            "Estimates relative poses for each lighthouse using epnp");
    button["pose_estimation_multi_lighthouse"]->setToolTip(
            "Estimates global object pose using known lighthouse poses and relative sensor locations");
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
    button["use_steamVR_lighthouse_poses"]->setToolTip("instead of using our own lighthouse frames, we listen to the frames published by vive_node");
    button["compare_to_steamVR"]->setToolTip("toggles recording of steam vr vive_controller1 tracking and all selected tracked objects");

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
    QObject::connect(button["pose_estimation_epnp"], SIGNAL(clicked()), this,
                     SLOT(startEstimateObjectPoseEPNP()));
    QObject::connect(button["pose_estimation_multi_lighthouse"], SIGNAL(clicked()), this,
                     SLOT(startEstimateObjectPoseMultiLighthouse()));
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
    QObject::connect(button["add_tracked_object"], SIGNAL(clicked()), this, SLOT(addTrackedObject()));
    QObject::connect(button["remove_tracked_object"], SIGNAL(clicked()), this, SLOT(removeTrackedObject()));
    QObject::connect(button["estimate_factory_calibration_values"], SIGNAL(clicked()), this, SLOT(estimateFactoryCalibration()));
//    QObject::connect(button["estimate_factory_calibration_values_2"], SIGNAL(clicked()), this, SLOT(estimateFactoryCalibration2()));
    QObject::connect(button["estimate_factory_calibration_values_epnp"], SIGNAL(clicked()), this, SLOT(estimateFactoryCalibrationEPNP()));
    QObject::connect(button["estimate_factory_calibration_values_multi"], SIGNAL(clicked()), this, SLOT(estimateFactoryCalibrationMulti()));
    QObject::connect(button["reset_factory_calibration_values"], SIGNAL(clicked()), this, SLOT(resetFactoryCalibration()));
    QObject::connect(button["reset_pose"], SIGNAL(clicked()), this, SLOT(resetPose()));
    QObject::connect(button["compare_to_steamVR"], SIGNAL(clicked()), this, SLOT(compareToSteamVR()));
    QObject::connect(button["lighthouse_use_phase_1"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));
    QObject::connect(button["lighthouse_use_tilt_1"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));
    QObject::connect(button["lighthouse_use_curve_1"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));
    QObject::connect(button["lighthouse_use_gibbous_1"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));
    QObject::connect(button["lighthouse_use_phase_2"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));
    QObject::connect(button["lighthouse_use_tilt_2"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));
    QObject::connect(button["lighthouse_use_curve_2"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));
    QObject::connect(button["lighthouse_use_gibbous_2"], SIGNAL(clicked()), this, SLOT(useViveCalibrationValues()));

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
    ui.update_frequencies_horizontal_lighthouse_1->yAxis->setRange(0, 100);
    ui.update_frequencies_horizontal_lighthouse_2->yAxis->setLabel("frequency[Hz]");
    ui.update_frequencies_horizontal_lighthouse_2->yAxis->setRange(0, 100);
    ui.update_frequencies_vertical_lighthouse_1->yAxis->setLabel("frequency[Hz]");
    ui.update_frequencies_vertical_lighthouse_1->yAxis->setRange(0, 100);
    ui.update_frequencies_vertical_lighthouse_2->yAxis->setLabel("frequency[Hz]");
    ui.update_frequencies_vertical_lighthouse_2->yAxis->setRange(0, 100);

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
    ros::shutdown();
}

void RoboyDarkRoom::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                 qt_gui_cpp::Settings &instance_settings) const {
}

void RoboyDarkRoom::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                    const qt_gui_cpp::Settings &instance_settings) {
}

void RoboyDarkRoom::alignToViveController(){
    int i = 0;
    for (auto it = trackedObjects.begin(); it != trackedObjects.end();++it,i++) {
        if (trackedObjectsInfo[i].selected->isChecked()) {
            if(button["align_to_vive_controller"]->isChecked()) {
//                if(getTransform("vive_controller1","")
            }else{
                
            }
        }
    }
}

void RoboyDarkRoom::compareToSteamVR(){
    int i = 0;
    for (auto it = trackedObjects.begin(); it != trackedObjects.end();++it,i++) {
        if (trackedObjectsInfo[i].selected->isChecked()) {
            if(button["compare_to_steamVR"]->isChecked()) {
                char str[100], t[20];
                time_t now = std::time(0);
                strftime(t, 20, "%d%m%Y_%H%M%S", localtime(&now));
                it->get()->mux.lock();
                sprintf(str, "record_TPE_%s_%s.log", it->get()->name.c_str(), t);
                it->get()->steamVRrecord[0].open(str);
                sprintf(str, "record_MLPE_%s_%s.log", it->get()->name.c_str(), t);
                it->get()->steamVRrecord[1].open(str);
                if (it->get()->steamVRrecord[0].is_open() && it->get()->steamVRrecord[1].is_open()) {
                    it->get()->steamVRrecord[0] << "time stamp[ns], \tx[VO], \ty, \tz, \tdx[m/s], \tdy, \tdz, \tqx, \tqy, \tqz, \tqw, \tx[VIVE], \ty, \tz, \tdx[m/s], \tdy, \tdz, \tqx, \tqy, \tqz, \tqw" << endl;
                    it->get()->steamVRrecord[1] << "time stamp[ns], \tx[VO], \ty, \tz, \tdx[m/s], \tdy, \tdz, \tqx, \tqy, \tqz, \tqw, \tx[VIVE], \ty, \tz, \tdx[m/s], \tdy, \tdz, \tqx, \tqy, \tqz, \tqw" << endl;
                    it->get()->comparesteamvr = true;
                }
                it->get()->mux.unlock();
            }else{
                it->get()->mux.lock();
                it->get()->comparesteamvr = false;
                it->get()->steamVRrecord[0].close();
                it->get()->steamVRrecord[1].close();
                it->get()->mux.unlock();
            }
        }
    }
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
        simulation.first->startIMUPublisher();
        simulation.first->startSensorPublisher();
        simulation.second->startSensorPublisher();

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
//    lighthouse1.setOrigin(tf::Vector3(0, -2.4, -0.115));
    lighthouse1.setOrigin(tf::Vector3(0, -1, 0));
    lighthouse2.setRotation(quat);
//    lighthouse2.setOrigin(tf::Vector3(-0.825, -2.4, -0.115));
    lighthouse2.setOrigin(tf::Vector3(-0.825, -1, 0));
}

void RoboyDarkRoom::record() {
    ROS_DEBUG("record clicked");
    if (trackedObjects.empty())
        button["record"]->setChecked(false);
    for (auto const &object:trackedObjects) {
        object->mux.lock();
        object->record(button["record"]->isChecked());
        object->mux.unlock();
    }
    for (auto const &simulation:lighthouse_simulation) {
        simulation.first->record(button["record"]->isChecked());
        simulation.second->record(button["record"]->isChecked());
    }
}

void RoboyDarkRoom::showRays() {
    ROS_DEBUG("show rays clicked");

    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
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
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::showDistances() {
    ROS_DEBUG("show distances clicked");
    for (auto const &object:trackedObjects) {
        object->mux.lock();
        object->distances = object->rays = button["show_distances"]->isChecked();
        object->mux.unlock();
    }
}

void RoboyDarkRoom::switchLighthouses() {
    ROS_DEBUG("switch lighthouses clicked");
    for (auto const &object:trackedObjects) {
        object->mux.lock();
        object->switchLighthouses(button["switch_lighthouses"]->isChecked());
        object->mux.unlock();
    }
}

void RoboyDarkRoom::startCalibrateRelativeSensorDistances() {
    ROS_DEBUG("calibrate_relative_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
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
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::startTriangulation() {
    ROS_DEBUG("triangulate clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
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
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::startPoseEstimationSensorCloud() {
    ROS_DEBUG("pose_correction_sensor_cloud clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        ROS_INFO("starting pose estimation thread");
        trackedObjects[i]->poseestimating = true;
        trackedObjects[i]->poseestimation_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->lighthousePoseEstimationLeastSquares();
                }));
        trackedObjects[i]->poseestimation_thread->detach();
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::startObjectPoseEstimationSensorCloud() {
    ROS_DEBUG("object pose estimation clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        if (button["object_pose_estimation_least_squares"]->isChecked()) {
            ROS_INFO("starting pose estimation thread");
            if(!button["triangulate"]->isChecked()){
                ROS_INFO("starting tracking thread");
                trackedObjects[i]->tracking = true;
                trackedObjects[i]->tracking_thread = boost::shared_ptr<boost::thread>(
                        new boost::thread(
                                [this, i]() { this->trackedObjects[i]->triangulateSensors(); }
                        ));
                trackedObjects[i]->tracking_thread->detach();
                button["triangulate"]->setChecked(true);
            }
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
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::startEstimateSensorPositionsUsingRelativeDistances() {
    ROS_DEBUG("position_estimation_relativ_sensor_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
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
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::startEstimateObjectPoseUsingRelativeDistances() {
    ROS_DEBUG("pose_estimation_relativ_sensor_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        ROS_INFO("starting relativ pose thread");
        trackedObjects[i]->relative_pose_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateObjectPoseUsingRelativeDistances();
                }));
        trackedObjects[i]->relative_pose_thread->detach();
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::startEstimateObjectPoseEPNP() {
    ROS_DEBUG("pose_estimation_epnp clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        ROS_INFO("starting relativ pose epnp thread");
        trackedObjects[i]->poseestimating_epnp = true;
        trackedObjects[i]->relative_pose_epnp_thread = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateObjectPoseEPNP();
                }));
        trackedObjects[i]->relative_pose_epnp_thread->detach();
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::startEstimateObjectPoseMultiLighthouse() {
    ROS_DEBUG("pose_estimation_multi_lighthouse clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        trackedObjects[i]->mux.lock();
        if(button["pose_estimation_multi_lighthouse"]->isChecked()) {
            ROS_INFO("starting multi lighthouse pose estimation thread");
            trackedObjects[i]->poseestimating_multiLighthouse = true;
            trackedObjects[i]->object_pose_estimation_multi_lighthouse_thread = boost::shared_ptr<boost::thread>(
                    new boost::thread([this, i]() {
                        this->trackedObjects[i]->estimateObjectPoseMultiLighthouse();
                    }));
            trackedObjects[i]->object_pose_estimation_multi_lighthouse_thread->detach();
        }else{
            trackedObjects[i]->poseestimating_multiLighthouse = false;
        }
        trackedObjects[i]->mux.unlock();
    }
}

void RoboyDarkRoom::transformPublisher() {
    ros::Rate rate(30);
    Vector3d pos(0,0,0), vel(0.3,0.3,0.3);
    double boundary = 0.5;
    while (publish_transform) {
        mux.lock();
        if(!button["use_steamVR_lighthouse_poses"]->isChecked()){
            tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
            tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
        }
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
                    object->pose.setOrigin( tf::Vector3(pos(0),pos(1),pos(2)));
                    tf::Quaternion q(0, 0, 0, 1);
                    q.setRPY(random_pose_roll, random_pose_pitch, random_pose_yaw);
                    object->pose.setRotation(q);
                    if(ui.random_pose_x->isChecked())
                        pos(0) += vel(0)*ui.random_pose_slider->value()/1000.0+(rand()/(double)RAND_MAX-0.5)*0.001;
                    if(ui.random_pose_y->isChecked())
                        pos(1) += vel(1)*ui.random_pose_slider->value()/1000.0+(rand()/(double)RAND_MAX-0.5)*0.001;
                    if(ui.random_pose_z->isChecked())
                        pos(2) += vel(2)*ui.random_pose_slider->value()/1000.0+(rand()/(double)RAND_MAX-0.5)*0.001;
                    if(ui.random_pose_roll->isChecked())
                        random_pose_roll += rand()/(double)RAND_MAX*ui.random_pose_slider->value()/1000.0;
                    if(ui.random_pose_pitch->isChecked())
                        random_pose_pitch += rand()/(double)RAND_MAX*ui.random_pose_slider->value()/1000.0;
                    if(ui.random_pose_yaw->isChecked())
                        random_pose_yaw += rand()/(double)RAND_MAX*ui.random_pose_slider->value()/1000.0;

                    if(pos(0)>boundary || pos(0)<-boundary)
                        vel(0) = -(vel(0)+(rand()/(double)RAND_MAX-0.5));
                    if(pos(1)>boundary || pos(1)<-boundary)
                        vel(1) = -(vel(1)+(rand()/(double)RAND_MAX-0.5));
                    if(pos(2)>boundary || pos(2)<-boundary)
                        vel(2) = -(vel(2)+(rand()/(double)RAND_MAX-0.5));
//                    object->pose.setOrigin( tf::Vector3(random_pose_x,random_pose_y,random_pose_z));
//                    tf::Quaternion q(0, 0, 0, 1);
//                    q.setRPY(random_pose_roll, random_pose_pitch, random_pose_yaw);
//                    object->pose.setRotation(q);
//                    if(ui.random_pose_x->isChecked())
//                        random_pose_x += 0.001 * (0.5-rand()/double(RAND_MAX));
//                    if(ui.random_pose_y->isChecked())
//                        random_pose_y += 0.001 * (0.5-rand()/double(RAND_MAX));
//                    if(ui.random_pose_z->isChecked())
//                        random_pose_z += 0.001 * (0.5-rand()/double(RAND_MAX));
//                    if(ui.random_pose_roll->isChecked())
//                        random_pose_roll += 0.001 * (0.5-rand()/double(RAND_MAX));
//                    if(ui.random_pose_pitch->isChecked())
//                        random_pose_pitch += 0.001 * (0.5-rand()/double(RAND_MAX));
//                    if(ui.random_pose_yaw->isChecked())
//                        random_pose_yaw += 0.001 * (0.5-rand()/double(RAND_MAX));
                }
            }
            for (auto &object:trackedObjects) {
                tf_broadcaster.sendTransform(tf::StampedTransform(object->pose, ros::Time::now(),
                                                                  "world", object->name.c_str()));
            }
//        }
        mux.unlock();
        rate.sleep();
    }
}

void RoboyDarkRoom::correctPose(const roboy_communication_middleware::LighthousePoseCorrection &msg) {
    mux.lock();
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
    mux.unlock();
}

void RoboyDarkRoom::interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg) {
    mux.lock();
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
    mux.unlock();
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
        switch(msg->mode){
            case 0: text["lighthouse_selected_mode_1"]->setText("A"); break;
            case 1: text["lighthouse_selected_mode_1"]->setText("B"); break;
            case 2: text["lighthouse_selected_mode_1"]->setText("C"); break;
        }
        text["lighthouse_fault_detect_flags_1"]->setText(QString::number(msg->faults));
        text["lighthouse_desync_counter_1"]->setText(QString::number(msg->unlock_count));
        text["lighthouse_acc_x_1"]->setText(QString::number(msg->accel_dir_x));
        text["lighthouse_acc_y_1"]->setText(QString::number(msg->accel_dir_y));
        text["lighthouse_acc_z_1"]->setText(QString::number(msg->accel_dir_z));
        calibration[LIGHTHOUSE_A][HORIZONTAL].phase = msg->fcal_0_phase;
        calibration[LIGHTHOUSE_A][VERTICAL].phase = msg->fcal_1_phase;
        calibration[LIGHTHOUSE_A][HORIZONTAL].tilt = -msg->fcal_0_tilt;
        calibration[LIGHTHOUSE_A][VERTICAL].tilt = -msg->fcal_0_tilt;
        calibration[LIGHTHOUSE_A][HORIZONTAL].curve = msg->fcal_0_curve;
        calibration[LIGHTHOUSE_A][VERTICAL].curve = msg->fcal_0_curve;
        calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase = msg->fcal_0_gibphase;
        calibration[LIGHTHOUSE_A][VERTICAL].gibphase = msg->fcal_1_gibphase;
        calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag = msg->fcal_0_gibmag;
        calibration[LIGHTHOUSE_A][VERTICAL].gibmag = msg->fcal_1_gibmag;
        text["lighthouse_phase0_1"]->setText(QString::number(msg->fcal_0_phase));
        text["lighthouse_phase1_1"]->setText(QString::number(msg->fcal_1_phase));
        text["lighthouse_tilt0_1"]->setText(QString::number(msg->fcal_0_tilt));
        text["lighthouse_tilt1_1"]->setText(QString::number(msg->fcal_1_tilt));
        text["lighthouse_curve0_1"]->setText(QString::number(msg->fcal_0_curve));
        text["lighthouse_curve1_1"]->setText(QString::number(msg->fcal_1_curve));
        text["lighthouse_gibphase0_1"]->setText(QString::number(msg->fcal_0_gibphase));
        text["lighthouse_gibphase1_1"]->setText(QString::number(msg->fcal_1_gibphase));
        text["lighthouse_gibmag0_1"]->setText(QString::number(msg->fcal_0_gibmag));
        text["lighthouse_gibmag1_1"]->setText(QString::number(msg->fcal_1_gibmag));
    } else {
        text["lighthouse_firmware_version_2"]->setText(QString::number(msg->fw_version >> 6 & 0x3FF, 16));
        text["lighthouse_protocol_version_2"]->setText(QString::number(msg->fw_version & 0x1F, 10));
        text["lighthouse_ID_2"]->setText(QString::number(msg->ID, 16));
        text["lighthouse_hardware_version_2"]->setText(QString::number(msg->hw_version, 16));
        switch(msg->mode){
            case 0: text["lighthouse_selected_mode_2"]->setText("A"); break;
            case 1: text["lighthouse_selected_mode_2"]->setText("B"); break;
            case 2: text["lighthouse_selected_mode_2"]->setText("C"); break;
        }
        text["lighthouse_fault_detect_flags_2"]->setText(QString::number(msg->faults));
        text["lighthouse_desync_counter_2"]->setText(QString::number(msg->unlock_count));
        text["lighthouse_acc_x_2"]->setText(QString::number(msg->accel_dir_x));
        text["lighthouse_acc_y_2"]->setText(QString::number(msg->accel_dir_y));
        text["lighthouse_acc_z_2"]->setText(QString::number(msg->accel_dir_z));
        calibration[LIGHTHOUSE_B][HORIZONTAL].phase = msg->fcal_0_phase;
        calibration[LIGHTHOUSE_B][VERTICAL].phase = msg->fcal_1_phase;
        calibration[LIGHTHOUSE_B][HORIZONTAL].tilt = -msg->fcal_0_tilt;
        calibration[LIGHTHOUSE_B][VERTICAL].tilt = -msg->fcal_0_tilt;
        calibration[LIGHTHOUSE_B][HORIZONTAL].curve = msg->fcal_0_curve;
        calibration[LIGHTHOUSE_B][VERTICAL].curve = msg->fcal_0_curve;
        calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase = msg->fcal_0_gibphase;
        calibration[LIGHTHOUSE_B][VERTICAL].gibphase = msg->fcal_1_gibphase;
        calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag = msg->fcal_0_gibmag;
        calibration[LIGHTHOUSE_B][VERTICAL].gibmag = msg->fcal_1_gibmag;
        text["lighthouse_phase0_2"]->setText(QString::number(msg->fcal_0_phase));
        text["lighthouse_phase1_2"]->setText(QString::number(msg->fcal_1_phase));
        text["lighthouse_tilt0_2"]->setText(QString::number(msg->fcal_0_tilt));
        text["lighthouse_tilt1_2"]->setText(QString::number(msg->fcal_1_tilt));
        text["lighthouse_curve0_2"]->setText(QString::number(msg->fcal_0_curve));
        text["lighthouse_curve1_2"]->setText(QString::number(msg->fcal_1_curve));
        text["lighthouse_gibphase0_2"]->setText(QString::number(msg->fcal_0_gibphase));
        text["lighthouse_gibphase1_2"]->setText(QString::number(msg->fcal_1_gibphase));
        text["lighthouse_gibmag0_2"]->setText(QString::number(msg->fcal_0_gibmag));
        text["lighthouse_gibmag1_2"]->setText(QString::number(msg->fcal_1_gibmag));
    }
    useViveCalibrationValues();
}

bool RoboyDarkRoom::fileExists(const string &filepath) {
    struct stat buffer;
    return (stat(filepath.c_str(), &buffer) == 0);
}

void RoboyDarkRoom::updateTrackedObjectInfo() {
    ros::Rate rate(1);
    while (update_tracked_object_info) {
        mux.lock();
        for (int i = 0; i < trackedObjects.size(); i++) {
            char str[100];
            sprintf(str, "%d/%d", trackedObjects[i]->active_sensors, (int) trackedObjects[i]->sensors.size());
            trackedObjectsInfo[i].activeSensors->setText(str);
        }
        mux.unlock();
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

bool RoboyDarkRoom::addTrackedObject(const char *config_file_path) {
    mux.lock();
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    if (strlen(config_file_path) == 0) {
        QModelIndexList indexList = ui.tracked_object_browser->selectionModel()->selectedIndexes();
        if (indexList.empty())
            return false;
        if (!newObject->init(model->filePath(indexList[0]).toStdString().c_str()))
            return false;
        if (ui.simulate->isChecked()) {
            pair<LighthouseSimulatorPtr, LighthouseSimulatorPtr> simulation;

            vector<fs::path> parts = {model->filePath(indexList[0]).toStdString().c_str()};

            simulation.first.reset(new LighthouseSimulator(LIGHTHOUSE_A, parts));
            simulation.second.reset(new LighthouseSimulator(LIGHTHOUSE_B, parts));
            simulation.first->startSensorPublisher();
            simulation.second->startSensorPublisher();
            lighthouse_simulation.push_back(simulation);
        }
    } else {
        if (!newObject->init(config_file_path))
            return false;
        if (ui.simulate->isChecked()) {
            pair<LighthouseSimulatorPtr, LighthouseSimulatorPtr> simulation;

            vector<fs::path> parts = {config_file_path};

            simulation.first.reset(new LighthouseSimulator(LIGHTHOUSE_A, parts));
            simulation.second.reset(new LighthouseSimulator(LIGHTHOUSE_B, parts));
            simulation.first->startSensorPublisher();
            simulation.second->startSensorPublisher();
            lighthouse_simulation.push_back(simulation);
        }
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
    mux.unlock();
    return true;
}

/**
 * removes the selected tracked object
 */
void RoboyDarkRoom::removeTrackedObject() {
    mux.lock();
    publish_transform = false;
    if (transform_thread->joinable()) {
        ROS_INFO("waiting for transform thread to shut down");
        transform_thread->join();
    }
    update_tracked_object_info = false;
    if (update_tracked_object_info_thread->joinable()) {
        ROS_INFO("waiting for update_tracked_object_info_thread to shut down");
        update_tracked_object_info_thread->join();
    }
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
    publish_transform = true;
    transform_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyDarkRoom::transformPublisher, this));
    transform_thread->detach();
    update_tracked_object_info = true;
    update_tracked_object_info_thread = boost::shared_ptr<std::thread>(
            new std::thread(&RoboyDarkRoom::updateTrackedObjectInfo, this));
    update_tracked_object_info_thread->detach();
    mux.unlock();
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

void RoboyDarkRoom::useViveCalibrationValues(){
    for(auto &object:trackedObjects){
        object->mux.lock();
        if(button["lighthouse_use_phase_1"]->isChecked()){
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].phase = calibration[LIGHTHOUSE_A][HORIZONTAL].phase;
            object->calibration[LIGHTHOUSE_A][VERTICAL].phase = calibration[LIGHTHOUSE_A][VERTICAL].phase;
        }else{
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].phase = 0;
            object->calibration[LIGHTHOUSE_A][VERTICAL].phase = 0;
        }
        if(button["lighthouse_use_tilt_1"]->isChecked()){
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].tilt = calibration[LIGHTHOUSE_A][HORIZONTAL].tilt;
            object->calibration[LIGHTHOUSE_A][VERTICAL].tilt = calibration[LIGHTHOUSE_A][VERTICAL].tilt;
        }else{
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].tilt = 0;
            object->calibration[LIGHTHOUSE_A][VERTICAL].tilt = 0;
        }
        if(button["lighthouse_use_curve_1"]->isChecked()){
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].curve = calibration[LIGHTHOUSE_A][HORIZONTAL].curve;
            object->calibration[LIGHTHOUSE_A][VERTICAL].curve = calibration[LIGHTHOUSE_A][VERTICAL].curve;
        }else{
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].curve = 0;
            object->calibration[LIGHTHOUSE_A][VERTICAL].curve = 0;
        }
        if(button["lighthouse_use_gibbous_1"]->isChecked()){
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase = calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase;
            object->calibration[LIGHTHOUSE_A][VERTICAL].gibphase = calibration[LIGHTHOUSE_A][VERTICAL].gibphase;
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag = calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag;
            object->calibration[LIGHTHOUSE_A][VERTICAL].gibmag = calibration[LIGHTHOUSE_A][VERTICAL].gibmag;
        }else{
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibphase = 0;
            object->calibration[LIGHTHOUSE_A][VERTICAL].gibphase = 0;
            object->calibration[LIGHTHOUSE_A][HORIZONTAL].gibmag = 0;
            object->calibration[LIGHTHOUSE_A][VERTICAL].gibmag = 0;
        }

        if(button["lighthouse_use_phase_2"]->isChecked()){
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].phase = calibration[LIGHTHOUSE_B][HORIZONTAL].phase;
            object->calibration[LIGHTHOUSE_B][VERTICAL].phase = calibration[LIGHTHOUSE_B][VERTICAL].phase;
        }else{
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].phase = 0;
            object->calibration[LIGHTHOUSE_B][VERTICAL].phase = 0;
        }
        if(button["lighthouse_use_tilt_2"]->isChecked()){
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].tilt = calibration[LIGHTHOUSE_B][HORIZONTAL].tilt;
            object->calibration[LIGHTHOUSE_B][VERTICAL].tilt = calibration[LIGHTHOUSE_B][VERTICAL].tilt;
        }else{
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].tilt = 0;
            object->calibration[LIGHTHOUSE_B][VERTICAL].tilt = 0;
        }
        if(button["lighthouse_use_curve_2"]->isChecked()){
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].curve = calibration[LIGHTHOUSE_B][HORIZONTAL].curve;
            object->calibration[LIGHTHOUSE_B][VERTICAL].curve = calibration[LIGHTHOUSE_B][VERTICAL].curve;
        }else{
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].curve = 0;
            object->calibration[LIGHTHOUSE_B][VERTICAL].curve = 0;
        }
        if(button["lighthouse_use_gibbous_2"]->isChecked()){
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase = calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase;
            object->calibration[LIGHTHOUSE_B][VERTICAL].gibphase = calibration[LIGHTHOUSE_B][VERTICAL].gibphase;
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag = calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag;
            object->calibration[LIGHTHOUSE_B][VERTICAL].gibmag = calibration[LIGHTHOUSE_B][VERTICAL].gibmag;
        }else{
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibphase = 0;
            object->calibration[LIGHTHOUSE_B][VERTICAL].gibphase = 0;
            object->calibration[LIGHTHOUSE_B][HORIZONTAL].gibmag = 0;
            object->calibration[LIGHTHOUSE_B][VERTICAL].gibmag = 0;
        }
        object->mux.unlock();
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

void RoboyDarkRoom::estimateFactoryCalibration2(){
    ROS_DEBUG("estimate_factory_calibration_2 clicked");
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

        simulation.first->startSensorPublisher();
        simulation.second->startSensorPublisher();

        lighthouse_simulation.push_back(simulation);
    }

    ROS_DEBUG_STREAM("adding tracked object " << calibration_object_path);
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->init(calibration_object_path.c_str());
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

    TrackedObjectPtr calibration_object = trackedObjects.back();
    calibration_object->rays = true;
    calibration_object->rays_thread = boost::shared_ptr<boost::thread>(
            new boost::thread(
                    [calibration_object]() { calibration_object->publishRays(); }
            ));
    calibration_object->rays_thread->detach();

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
        if(calibration_object->estimateFactoryCalibration2(LIGHTHOUSE_A)){
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
        if(calibration_object->estimateFactoryCalibration2(LIGHTHOUSE_B)){
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

void RoboyDarkRoom::estimateFactoryCalibrationEPNP(){
    ROS_DEBUG("estimate_factory_calibration_epnp clicked");
    string package_path = ros::package::getPath("darkroom");
    string calibration_object_path = package_path + "/calibrated_objects/sphereTracker18cm.yaml";
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

        simulation.first->startSensorPublisher();
        simulation.second->startSensorPublisher();

        lighthouse_simulation.push_back(simulation);
    }

    ROS_DEBUG_STREAM("adding tracked object " << calibration_object_path);
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->init(calibration_object_path.c_str());
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

    TrackedObjectPtr calibration_object = trackedObjects.back();
    calibration_object->rays = true;
    calibration_object->rays_thread = boost::shared_ptr<boost::thread>(
            new boost::thread(
                    [calibration_object]() { calibration_object->publishRays(); }
            ));
    calibration_object->rays_thread->detach();

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
        if(calibration_object->estimateFactoryCalibrationEPNP(LIGHTHOUSE_A)){
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
        if(calibration_object->estimateFactoryCalibrationEPNP(LIGHTHOUSE_B)){
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

void RoboyDarkRoom::estimateFactoryCalibrationMulti(){
    ROS_DEBUG("estimate_factory_calibration_multi clicked");
    string package_path = ros::package::getPath("darkroom");
    string calibration_object_path = package_path + "/calibrated_objects/sphereTracker18cm.yaml";
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

        simulation.first->startSensorPublisher();
        simulation.second->startSensorPublisher();

        lighthouse_simulation.push_back(simulation);
    }

    ROS_DEBUG_STREAM("adding tracked object " << calibration_object_path);
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->init(calibration_object_path.c_str());
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

    TrackedObjectPtr calibration_object = trackedObjects.back();
//    calibration_object->rays = true;
//    calibration_object->rays_thread = boost::shared_ptr<boost::thread>(
//            new boost::thread(
//                    [calibration_object]() { calibration_object->publishRays(); }
//            ));
//    calibration_object->rays_thread->detach();

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
        if(calibration_object->estimateFactoryCalibrationMultiLighthouse(LIGHTHOUSE_A)){
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
        if(calibration_object->estimateFactoryCalibrationMultiLighthouse(LIGHTHOUSE_B)){
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
        object->mux.lock();
        object->calibration[LIGHTHOUSE_A][HORIZONTAL].reset();
        object->calibration[LIGHTHOUSE_A][VERTICAL].reset();
        object->calibration[LIGHTHOUSE_B][HORIZONTAL].reset();
        object->calibration[LIGHTHOUSE_B][VERTICAL].reset();
        object->mux.unlock();
    }
    if(!lighthouse_simulation.empty()){
        for(auto &lighthouse:lighthouse_simulation){
            lighthouse.first->calibration[LIGHTHOUSE_A][HORIZONTAL].reset();
            lighthouse.first->calibration[LIGHTHOUSE_A][VERTICAL].reset();
            lighthouse.second->calibration[LIGHTHOUSE_B][HORIZONTAL].reset();
            lighthouse.second->calibration[LIGHTHOUSE_B][VERTICAL].reset();
        }
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
