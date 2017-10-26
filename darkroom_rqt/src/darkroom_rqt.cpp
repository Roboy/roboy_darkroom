#include <darkroom_rqt/darkroom_rqt.hpp>

tf::Transform RoboyDarkRoom::lighthouse1;
tf::Transform RoboyDarkRoom::lighthouse2;
tf::Transform RoboyDarkRoom::simulated_object_lighthouse1;
tf::Transform RoboyDarkRoom::simulated_object_lighthouse2;
tf::Transform RoboyDarkRoom::tf_world;

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

    text["broadcast_ip"] = widget_->findChild<QLineEdit *>("broadcast_ip");
    text["broadcast_port"] = widget_->findChild<QLineEdit *>("broadcast_port");
    text["load_object"] = widget_->findChild<QLineEdit *>("load_object_text");

    slider["lighthouse1_x"] = widget_->findChild<QSlider *>("lighthouse1_x");
    slider["lighthouse1_y"] = widget_->findChild<QSlider *>("lighthouse1_y");
    slider["lighthouse1_z"] = widget_->findChild<QSlider *>("lighthouse1_z");
    slider["lighthouse1_roll"] = widget_->findChild<QSlider *>("lighthouse1_roll");
    slider["lighthouse1_pitch"] = widget_->findChild<QSlider *>("lighthouse1_pitch");
    slider["lighthouse1_yaw"] = widget_->findChild<QSlider *>("lighthouse1_yaw");

    slider["lighthouse2_x"] = widget_->findChild<QSlider *>("lighthouse2_x");
    slider["lighthouse2_y"] = widget_->findChild<QSlider *>("lighthouse2_y");
    slider["lighthouse2_z"] = widget_->findChild<QSlider *>("lighthouse2_z");
    slider["lighthouse2_roll"] = widget_->findChild<QSlider *>("lighthouse2_roll");
    slider["lighthouse2_pitch"] = widget_->findChild<QSlider *>("lighthouse2_pitch");
    slider["lighthouse2_yaw"] = widget_->findChild<QSlider *>("lighthouse2_yaw");

    slider["simulatedObject_x_lighthouse_1"] = widget_->findChild<QSlider *>("simulatedObject_x_lighthouse_1");
    slider["simulatedObject_y_lighthouse_1"] = widget_->findChild<QSlider *>("simulatedObject_y_lighthouse_1");
    slider["simulatedObject_z_lighthouse_1"] = widget_->findChild<QSlider *>("simulatedObject_z_lighthouse_1");
    slider["simulatedObject_roll_lighthouse_1"] = widget_->findChild<QSlider *>("simulatedObject_roll_lighthouse_1");
    slider["simulatedObject_pitch_lighthouse_1"] = widget_->findChild<QSlider *>("simulatedObject_pitch_lighthouse_1");
    slider["simulatedObject_yaw_lighthouse_1"] = widget_->findChild<QSlider *>("simulatedObject_yaw_lighthouse_1");

    slider["simulatedObject_x_lighthouse_2"] = widget_->findChild<QSlider *>("simulatedObject_x_lighthouse_2");
    slider["simulatedObject_y_lighthouse_2"] = widget_->findChild<QSlider *>("simulatedObject_y_lighthouse_2");
    slider["simulatedObject_z_lighthouse_2"] = widget_->findChild<QSlider *>("simulatedObject_z_lighthouse_2");
    slider["simulatedObject_roll_lighthouse_2"] = widget_->findChild<QSlider *>("simulatedObject_roll_lighthouse_2");
    slider["simulatedObject_pitch_lighthouse_2"] = widget_->findChild<QSlider *>("simulatedObject_pitch_lighthouse_2");
    slider["simulatedObject_yaw_lighthouse_2"] = widget_->findChild<QSlider *>("simulatedObject_yaw_lighthouse_2");

    button["triangulate"] = widget_->findChild<QPushButton *>("triangulate");
    button["show_rays"] = widget_->findChild<QPushButton *>("show_rays");
    button["show_distances"] = widget_->findChild<QPushButton *>("show_distances");
    button["pose_correction_sensor_cloud"] = widget_->findChild<QPushButton *>("pose_correction_sensor_cloud");
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
    button["object_pose_estimation_epnp"] = widget_->findChild<QPushButton *>("object_pose_estimation_epnp");
    button["object_pose_estimation_p3p"] = widget_->findChild<QPushButton *>("object_pose_estimation_p3p");
    button["load_object"] = widget_->findChild<QPushButton *>("load_object");

    button["triangulate"]->setToolTip(
            "activates triangulation of lighthouse rays\n"
                    "to get the 3d Position of visible sensors");
    button["show_rays"]->setToolTip("enable to visualize the lighthouse rays");
    button["show_distances"]->setToolTip("enable to visualize the distances\nbetween triangulated sensor positions");
    button["pose_correction_sensor_cloud"]->setToolTip(
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
    button["object_pose_estimation_epnp"]->setToolTip("estimates object pose using epnp");
    button["object_pose_estimation_p3p"]->setToolTip("estimates object pose using p3p");
    button["load_object"]->setToolTip("loads the given object yaml file");

    QObject::connect(slider["lighthouse1_x"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse1_y"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse1_z"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse1_roll"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse1_pitch"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse1_yaw"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));

    QObject::connect(slider["lighthouse2_x"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_y"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_z"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_roll"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_pitch"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));
    QObject::connect(slider["lighthouse2_yaw"], SIGNAL(valueChanged(int)), this, SLOT(resetLighthousePoses()));

    QObject::connect(slider["simulatedObject_x_lighthouse_1"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_y_lighthouse_1"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_z_lighthouse_1"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_roll_lighthouse_1"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_pitch_lighthouse_1"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_yaw_lighthouse_1"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));

    QObject::connect(slider["simulatedObject_x_lighthouse_2"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_y_lighthouse_2"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_z_lighthouse_2"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_roll_lighthouse_2"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_pitch_lighthouse_2"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));
    QObject::connect(slider["simulatedObject_yaw_lighthouse_2"], SIGNAL(valueChanged(int)), this,
                     SLOT(resetObjectPoses()));

    QObject::connect(button["triangulate"], SIGNAL(clicked()), this, SLOT(startTriangulation()));
    QObject::connect(button["show_rays"], SIGNAL(clicked()), this, SLOT(showRays()));
    QObject::connect(button["show_distances"], SIGNAL(clicked()), this, SLOT(showDistances()));
    QObject::connect(button["pose_correction_sensor_cloud"], SIGNAL(clicked()), this,
                     SLOT(startPoseEstimationSensorCloud()));
    QObject::connect(button["pose_correction_particle_filter"], SIGNAL(clicked()), this,
                     SLOT(startPoseEstimationParticleFilter()));
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
    QObject::connect(button["object_pose_estimation_epnp"], SIGNAL(clicked()), this, SLOT(startPoseEstimationEPnP()));
    QObject::connect(button["object_pose_estimation_p3p"], SIGNAL(clicked()), this, SLOT(startPoseEstimationP3P()));
    QObject::connect(button["load_object"], SIGNAL(clicked()), this, SLOT(loadObject()));

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "darkroom_rqt_plugin");
    }

    pose_correction_sub = nh->subscribe("/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1,
                                        &RoboyDarkRoom::correctPose, this);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    resetLighthousePoses();

    publish_transform = true;
    if (transform_thread == nullptr) {
        transform_thread = boost::shared_ptr<std::thread>(new std::thread(&RoboyDarkRoom::transformPublisher, this));
        transform_thread->detach();
    }
}

void RoboyDarkRoom::shutdownPlugin() {
}

void RoboyDarkRoom::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                 qt_gui_cpp::Settings &instance_settings) const {
    instance_settings.setValue("lighthouse1_x", slider["lighthouse1_x"]->value());
    instance_settings.setValue("lighthouse1_y", slider["lighthouse1_y"]->value());
    instance_settings.setValue("lighthouse1_z", slider["lighthouse1_z"]->value());
    instance_settings.setValue("lighthouse1_roll", slider["lighthouse1_roll"]->value());
    instance_settings.setValue("lighthouse1_pitch", slider["lighthouse1_pitch"]->value());
    instance_settings.setValue("lighthouse1_yaw", slider["lighthouse1_yaw"]->value());

    instance_settings.setValue("lighthouse2_x", slider["lighthouse2_x"]->value());
    instance_settings.setValue("lighthouse2_y", slider["lighthouse2_y"]->value());
    instance_settings.setValue("lighthouse2_z", slider["lighthouse2_z"]->value());
    instance_settings.setValue("lighthouse2_roll", slider["lighthouse2_roll"]->value());
    instance_settings.setValue("lighthouse2_pitch", slider["lighthouse2_pitch"]->value());
    instance_settings.setValue("lighthouse2_yaw", slider["lighthouse2_yaw"]->value());

    instance_settings.setValue("simulatedObject_x_lighthouse_1", slider["simulatedObject_x_lighthouse_1"]->value());
    instance_settings.setValue("simulatedObject_y_lighthouse_1", slider["simulatedObject_y_lighthouse_1"]->value());
    instance_settings.setValue("simulatedObject_z_lighthouse_1", slider["simulatedObject_z_lighthouse_1"]->value());
    instance_settings.setValue("simulatedObject_roll_lighthouse_1",
                               slider["simulatedObject_roll_lighthouse_1"]->value());
    instance_settings.setValue("simulatedObject_pitch_lighthouse_1",
                               slider["simulatedObject_pitch_lighthouse_1"]->value());
    instance_settings.setValue("simulatedObject_yaw_lighthouse_1", slider["simulatedObject_yaw_lighthouse_1"]->value());

    instance_settings.setValue("simulatedObject_x_lighthouse_2", slider["simulatedObject_x_lighthouse_2"]->value());
    instance_settings.setValue("simulatedObject_y_lighthouse_2", slider["simulatedObject_y_lighthouse_2"]->value());
    instance_settings.setValue("simulatedObject_z_lighthouse_2", slider["simulatedObject_z_lighthouse_2"]->value());
    instance_settings.setValue("simulatedObject_roll_lighthouse_2",
                               slider["simulatedObject_roll_lighthouse_2"]->value());
    instance_settings.setValue("simulatedObject_pitch_lighthouse_2",
                               slider["simulatedObject_pitch_lighthouse_2"]->value());
    instance_settings.setValue("simulatedObject_yaw_lighthouse_2", slider["simulatedObject_yaw_lighthouse_2"]->value());
    instance_settings.setValue("load_object", text["load_object"]->text());
}

void RoboyDarkRoom::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                    const qt_gui_cpp::Settings &instance_settings) {
    bool ok;
    slider["lighthouse1_x"]->setValue(instance_settings.value("lighthouse1_x").toInt());
    slider["lighthouse1_y"]->setValue(instance_settings.value("lighthouse1_y").toInt());
    slider["lighthouse1_z"]->setValue(instance_settings.value("lighthouse1_z").toInt());
    slider["lighthouse1_roll"]->setValue(instance_settings.value("lighthouse1_roll").toInt());
    slider["lighthouse1_pitch"]->setValue(instance_settings.value("lighthouse1_pitch").toInt());
    slider["lighthouse1_yaw"]->setValue(instance_settings.value("lighthouse1_yaw").toInt());

    slider["lighthouse2_x"]->setValue(instance_settings.value("lighthouse2_x").toInt());
    slider["lighthouse2_y"]->setValue(instance_settings.value("lighthouse2_y").toInt());
    slider["lighthouse2_z"]->setValue(instance_settings.value("lighthouse2_z").toInt());
    slider["lighthouse2_roll"]->setValue(instance_settings.value("lighthouse2_roll").toInt());
    slider["lighthouse2_pitch"]->setValue(instance_settings.value("lighthouse2_pitch").toInt());
    slider["lighthouse2_yaw"]->setValue(instance_settings.value("lighthouse2_yaw").toInt());

    slider["simulatedObject_x_lighthouse_1"]->setValue(
            instance_settings.value("simulatedObject_x_lighthouse_1").toInt());
    slider["simulatedObject_y_lighthouse_1"]->setValue(
            instance_settings.value("simulatedObject_y_lighthouse_1").toInt());
    slider["simulatedObject_z_lighthouse_1"]->setValue(
            instance_settings.value("simulatedObject_z_lighthouse_1").toInt());
    slider["simulatedObject_roll_lighthouse_1"]->setValue(
            instance_settings.value("simulatedObject_roll_lighthouse_1").toInt());
    slider["simulatedObject_pitch_lighthouse_1"]->setValue(
            instance_settings.value("simulatedObject_pitch_lighthouse_1").toInt());
    slider["simulatedObject_yaw_lighthouse_1"]->setValue(
            instance_settings.value("simulatedObject_yaw_lighthouse_1").toInt());

    slider["simulatedObject_x_lighthouse_2"]->setValue(
            instance_settings.value("simulatedObject_x_lighthouse_2").toInt());
    slider["simulatedObject_y_lighthouse_2"]->setValue(
            instance_settings.value("simulatedObject_y_lighthouse_2").toInt());
    slider["simulatedObject_z_lighthouse_2"]->setValue(
            instance_settings.value("simulatedObject_z_lighthouse_2").toInt());
    slider["simulatedObject_roll_lighthouse_2"]->setValue(
            instance_settings.value("simulatedObject_roll_lighthouse_2").toInt());
    slider["simulatedObject_pitch_lighthouse_2"]->setValue(
            instance_settings.value("simulatedObject_pitch_lighthouse_2").toInt());
    slider["simulatedObject_yaw_lighthouse_2"]->setValue(
            instance_settings.value("simulatedObject_yaw_lighthouse_2").toInt());
    text["load_object"]->setText( instance_settings.value("load_object").toString());
}

void RoboyDarkRoom::connectRoboy() {
    ROS_DEBUG("connect roboy clicked");
    TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
    newObject->connectRoboy();
    trackedObjects[object_counter] = newObject;
    object_counter++;
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
    text["lighthouse1_x"]->setText(QString::number(slider["lighthouse1_x"]->value() / 100.0 * 5.0));
    text["lighthouse1_y"]->setText(QString::number(slider["lighthouse1_y"]->value() / 100.0 * 5.0));
    text["lighthouse1_z"]->setText(QString::number(slider["lighthouse1_z"]->value() / 100.0 * 5.0));
    text["lighthouse1_roll"]->setText(QString::number(slider["lighthouse1_roll"]->value()));
    text["lighthouse1_pitch"]->setText(QString::number(slider["lighthouse1_pitch"]->value()));
    text["lighthouse1_yaw"]->setText(QString::number(slider["lighthouse1_yaw"]->value()));

    text["lighthouse2_x"]->setText(QString::number(slider["lighthouse2_x"]->value() / 100.0 * 5.0));
    text["lighthouse2_y"]->setText(QString::number(slider["lighthouse2_y"]->value() / 100.0 * 5.0));
    text["lighthouse2_z"]->setText(QString::number(slider["lighthouse2_z"]->value() / 100.0 * 5.0));
    text["lighthouse2_roll"]->setText(QString::number(slider["lighthouse2_roll"]->value()));
    text["lighthouse2_pitch"]->setText(QString::number(slider["lighthouse2_pitch"]->value()));
    text["lighthouse2_yaw"]->setText(QString::number(slider["lighthouse2_yaw"]->value()));

    tf_world.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion quat;
    quat.setRPY(0, 0, 0);
    tf_world.setRotation(quat);

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
    text["simulatedObject_x_lighthouse_1"]->setText(
            QString::number(slider["simulatedObject_x_lighthouse_1"]->value() / 100.0 * 5.0));
    text["simulatedObject_y_lighthouse_1"]->setText(
            QString::number(slider["simulatedObject_y_lighthouse_1"]->value() / 100.0 * 5.0));
    text["simulatedObject_z_lighthouse_1"]->setText(
            QString::number(slider["simulatedObject_z_lighthouse_1"]->value() / 100.0 * 5.0));
    text["simulatedObject_roll_lighthouse_1"]->setText(
            QString::number(slider["simulatedObject_roll_lighthouse_1"]->value()));
    text["simulatedObject_pitch_lighthouse_1"]->setText(
            QString::number(slider["simulatedObject_pitch_lighthouse_1"]->value()));
    text["simulatedObject_yaw_lighthouse_1"]->setText(
            QString::number(slider["simulatedObject_yaw_lighthouse_1"]->value()));

    text["simulatedObject_x_lighthouse_2"]->setText(
            QString::number(slider["simulatedObject_x_lighthouse_2"]->value() / 100.0 * 5.0));
    text["simulatedObject_y_lighthouse_2"]->setText(
            QString::number(slider["simulatedObject_y_lighthouse_2"]->value() / 100.0 * 5.0));
    text["simulatedObject_z_lighthouse_2"]->setText(
            QString::number(slider["simulatedObject_z_lighthouse_2"]->value() / 100.0 * 5.0));
    text["simulatedObject_roll_lighthouse_2"]->setText(
            QString::number(slider["simulatedObject_roll_lighthouse_2"]->value()));
    text["simulatedObject_pitch_lighthouse_2"]->setText(
            QString::number(slider["simulatedObject_pitch_lighthouse_2"]->value()));
    text["simulatedObject_yaw_lighthouse_2"]->setText(
            QString::number(slider["simulatedObject_yaw_lighthouse_2"]->value()));

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
                    ROS_INFO("Waiting for tracking thread to terminate");
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
                    this->trackedObjects[i]->poseEstimationSensorCloud();
                }));
        trackedObjects[i]->poseestimation_thread->detach();
    }
}

void RoboyDarkRoom::startEstimateSensorPositionsUsingRelativeDistances() {
    ROS_DEBUG("position_estimation_relativ_sensor_distances clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        ROS_INFO("starting relativ distance thread for lighthouse 1");
        trackedObjects[i]->distance_thread_1 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A,
                                                                                           trackedObjects[i]->calibrated_sensors);
                }));
        trackedObjects[i]->distance_thread_1->detach();
        ROS_INFO("starting relativ distance thread for lighthouse 2");
        trackedObjects[i]->distance_thread_2 = boost::shared_ptr<boost::thread>(
                new boost::thread([this, i]() {
                    this->trackedObjects[i]->estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B,
                                                                                           trackedObjects[i]->calibrated_sensors);
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

void RoboyDarkRoom::startPoseEstimationEPnP(){
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

void RoboyDarkRoom::startPoseEstimationP3P(){
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

void RoboyDarkRoom::loadObject(){
    ROS_DEBUG("load_object clicked");
    for (uint i = 0; i < trackedObjects.size(); i++) {
        lock_guard<mutex>(trackedObjects[i]->mux);
        trackedObjects[i]->readConfig(trackedObjects[i]->path + "/" + text["load_object"]->text().toStdString());
    }
}

void RoboyDarkRoom::transformPublisher() {
    ros::Rate rate(30);
    while (publish_transform) {
        lock_guard<mutex> lock(mux);
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
        tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
        if (simulate) {
            tf_broadcaster.sendTransform(
                    tf::StampedTransform(simulated_object_lighthouse1, ros::Time::now(), "lighthouse1",
                                         "simulated_object_lighthouse1"));
            tf_broadcaster.sendTransform(
                    tf::StampedTransform(simulated_object_lighthouse2, ros::Time::now(), "lighthouse2",
                                         "simulated_object_lighthouse2"));
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

PLUGINLIB_DECLARE_CLASS(roboy_darkroom, RoboyDarkRoom, RoboyDarkRoom, rqt_gui_cpp::Plugin)
