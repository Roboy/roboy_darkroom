#include "include/darkroom/DarkRoom.hpp"

namespace DarkRoomRviz {

    tf::Transform DarkRoom::lighthouse1;
    tf::Transform DarkRoom::lighthouse2;
    tf::Transform DarkRoom::tf_world;

    DarkRoom::DarkRoom(QWidget *parent)
            : rviz::Panel(parent) {

        // Create the main layout
        QHBoxLayout *mainLayout = new QHBoxLayout;

        // Create the frame to hold all the widgets
        QFrame *mainFrame = new QFrame();

        QVBoxLayout *frameLayout = new QVBoxLayout();

        QTabWidget *tabPage = new QTabWidget;

        // ################################################### connect TAB
        QWidget *connectWidget = new QWidget;
        connectWidget->setLayout(new QVBoxLayout);
        tabPage->addTab(connectWidget, "connect");

        QListWidget *listWidget = new QListWidget;
        listWidget->setObjectName("trackedObjects");
        connectWidget->layout()->addWidget(listWidget);

        // QTableWidget *tableWidget = new QTableWidget(4, 4);
        //
        // tableWidget->setItemDelegate(new TrackedObjectDelegate);
        // tableWidget->setEditTriggers(QAbstractItemView::DoubleClicked
        //                             | QAbstractItemView::SelectedClicked);
        // tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
        //
        // tableWidget->resize(500, 300);
        // TrackedObjectDelegate *item = new TrackedObjectDelegate;
        // populateTableWidget(tableWidget);
        // connectWidget->layout()->addWidget(tableWidget);

        QLabel *IP_label = new QLabel("IP:");
        connectWidget->layout()->addWidget(IP_label);
        QLineEdit *IP_line = new QLineEdit("192.168.0.107");
        IP_line->setObjectName("IP");
        connectWidget->layout()->addWidget(IP_line);

        QLabel *sensor_port_label = new QLabel("client_port:");
        connectWidget->layout()->addWidget(sensor_port_label);
        QLineEdit *sensor_port_line = new QLineEdit("4210");
        sensor_port_line->setObjectName("client_port");
        connectWidget->layout()->addWidget(sensor_port_line);

        QLabel *lighthousedistance_label = new QLabel("lighthouse distance:");
        connectWidget->layout()->addWidget(lighthousedistance_label);
        QLineEdit *lighthouse_distance = new QLineEdit("2,0");
        lighthouse_distance->setObjectName("lighthouse_distance");
        connectWidget->layout()->addWidget(lighthouse_distance);

        QPushButton *connect_button = new QPushButton(tr("connect"));
        connect(connect_button, SIGNAL(clicked()), this, SLOT(connectTo()));
        connectWidget->layout()->addWidget(connect_button);

        QCheckBox *listen = new QCheckBox(tr("listen for new objects"));
        listen->setObjectName("listen");
        connect(listen, SIGNAL(clicked()), this, SLOT(startObjectListener()));
        connectWidget->layout()->addWidget(listen);

        QCheckBox *trackIT = new QCheckBox(tr("triangulate"));
        trackIT->setObjectName("triangulate");
        connect(trackIT, SIGNAL(clicked()), this, SLOT(startTriangulation()));
        connectWidget->layout()->addWidget(trackIT);

        QCheckBox *rays = new QCheckBox(tr("rays"));
        rays->setObjectName("rays");
        connect(rays, SIGNAL(clicked()), this, SLOT(showRays()));
        connectWidget->layout()->addWidget(rays);

        QCheckBox *distances = new QCheckBox(tr("distances"));
        distances->setObjectName("distances");
        connect(distances, SIGNAL(clicked()), this, SLOT(showDistances()));
        connectWidget->layout()->addWidget(distances);

        QPushButton *switch_lighthouses_button = new QPushButton(tr("switch lighthouses"));
        connect(switch_lighthouses_button, SIGNAL(clicked()), this, SLOT(switch_lighthouses()));
        switch_lighthouses_button->setCheckable(true);
        switch_lighthouses_button->setObjectName("switch_lighthouses");
        connectWidget->layout()->addWidget(switch_lighthouses_button);

        QPushButton *calibrate_button = new QPushButton(tr("calibrate"));
        connect(calibrate_button, SIGNAL(clicked()), this, SLOT(calibrate()));
        connectWidget->layout()->addWidget(calibrate_button);

        QPushButton *estimateDistance_button = new QPushButton(tr("estimateDistance"));
        connect(estimateDistance_button, SIGNAL(clicked()), this, SLOT(startEstimateSensorPositionsUsingRelativeDistances()));
        connectWidget->layout()->addWidget(estimateDistance_button);

        QPushButton *estimatePose_button = new QPushButton(tr("poseestimation_sensor_cloud"));
        connect(estimatePose_button, SIGNAL(clicked()), this, SLOT(startPoseEstimationSensorCloud()));
        connectWidget->layout()->addWidget(estimatePose_button);

        QPushButton *estimatePose2_button = new QPushButton(tr("estimatePose2"));
        connect(estimatePose2_button, SIGNAL(clicked()), this, SLOT(estimatePose2()));
        connectWidget->layout()->addWidget(estimatePose2_button);

        QPushButton *particle_filter = new QPushButton(tr("particle filter"));
        connect(particle_filter, SIGNAL(clicked()), this, SLOT(startPoseEstimationParticleFilter()));
        connectWidget->layout()->addWidget(particle_filter);

        QPushButton *clearall_button = new QPushButton(tr("clear all"));
        connect(clearall_button, SIGNAL(clicked()), this, SLOT(clearAll()));
        connectWidget->layout()->addWidget(clearall_button);

        QPushButton *reset_ligthouse_poses_button = new QPushButton(tr("reset lighthouse poses"));
        connect(reset_ligthouse_poses_button, SIGNAL(clicked()), this, SLOT(resetLighthousePoses()));
        connectWidget->layout()->addWidget(reset_ligthouse_poses_button);

        QCheckBox *recordIT = new QCheckBox(tr("record"));
        recordIT->setObjectName("record");
        connect(recordIT, SIGNAL(clicked()), this, SLOT(record()));
        connectWidget->layout()->addWidget(recordIT);

        QPushButton *rebootESP_button = new QPushButton(tr("reboot ESP"));
        connect(rebootESP_button, SIGNAL(clicked()), this, SLOT(rebootESP()));
        connectWidget->layout()->addWidget(rebootESP_button);

        // ################################################### objects TAB
        QWidget *objectsWidget = new QWidget;
        objectsWidget->setLayout(new QVBoxLayout);
        QScrollArea *objects_scrollArea = new QScrollArea;
        objects_scrollArea->setWidgetResizable(true);

//    QListWidget *listWidget = new QListWidget;
//    listWidget->setObjectName("trackedObjects");
//    objects_scrollArea->setWidget(listWidget);
//    objectsWidget->layout()->addWidget(objects_scrollArea);
//    tabPage->addTab(objectsWidget, "objects");

        frameLayout->addWidget(tabPage);

        // Add frameLayout to the frame
        mainFrame->setLayout(frameLayout);

        // Add the frame to the main layout
        mainLayout->addWidget(mainFrame);

        // Remove margins to reduce space
        frameLayout->setContentsMargins(0, 0, 0, 0);
        mainLayout->setContentsMargins(0, 0, 0, 0);

        this->setLayout(mainLayout);

        // initialize ros

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "DarkRoomPlugin",
                      ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }

        nh = ros::NodeHandlePtr(new ros::NodeHandle);

        spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));

        visualization_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 100);

        pose_correction_sub = nh->subscribe("/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1,
                                            &DarkRoom::correctPose, this);

        resetLighthousePoses();

        TrackedObjectPtr newObject = TrackedObjectPtr(new TrackedObject());
        newObject->connectRoboy();
        trackedObjects[object_counter] = newObject;
        object_counter++;
        QListWidgetItem *item = new QListWidgetItem;
        item->setText("Roboy");
        this->findChild<QListWidget *>("trackedObjects")->addItem(item);
        ROS_INFO_STREAM("added Roboy");

        publish_transform = true;
        if(transform_thread==nullptr){
            transform_thread = boost::shared_ptr<std::thread>(new std::thread(&DarkRoom::transformPublisher, this));
            transform_thread->detach();
        }

        trackIT->click();
    }

    DarkRoom::~DarkRoom() {
        if(transform_thread!=nullptr) {
            publish_transform = false;
            if (transform_thread->joinable()) {
                ROS_INFO("waiting for transform thread to shut down");
                transform_thread->join();
            }
        }
    }

    void DarkRoom::save(rviz::Config config) const {
        QLineEdit *w = this->findChild<QLineEdit *>("IP");
        config.mapSetValue(w->objectName(), w->text());
        w = this->findChild<QLineEdit *>("client_port");
        config.mapSetValue(w->objectName(), w->text());
        rviz::Panel::save(config);
    }

    void DarkRoom::load(const rviz::Config &config) {
        rviz::Panel::load(config);
        QLineEdit *w = this->findChild<QLineEdit *>("IP");
        QVariant text;
        config.mapGetValue(w->objectName(), &text);
        w->setText(text.toString());
        w = this->findChild<QLineEdit *>("client_port");
        config.mapGetValue(w->objectName(), &text);
        w->setText(text.toString());
    }

    void DarkRoom::clearAll() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.id = message_counter++;
        message_counter = 0;
        marker.action = visualization_msgs::Marker::DELETEALL;
        visualization_pub.publish(marker);
    }

    void DarkRoom::resetLighthousePoses() {
        QLineEdit *w = this->findChild<QLineEdit *>("lighthouse_distance");
        float ligthouse_distance = atof(w->text().toStdString().c_str());

        tf_world.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion quat;
        quat.setRPY(0, 0, 0);
        tf_world.setRotation(quat);
        quat.setRPY(0, 0, 0);
        lighthouse1.setOrigin(tf::Vector3(0, 0, 0));
        lighthouse1.setRotation(quat);
        quat.setRPY(0, 0, 0);
        lighthouse2.setOrigin(tf::Vector3(ligthouse_distance, 0, 0));
        lighthouse2.setRotation(quat);
    }

    void DarkRoom::record() {
        QCheckBox *w = this->findChild<QCheckBox *>("record");
        if (trackedObjects.empty())
            w->setChecked(false);
        for (auto const &object:trackedObjects) {
            lock_guard<mutex> (object.second->mux);
            object.second->record(w->isChecked());
        }
    }

    void DarkRoom::showRays() {
        QCheckBox *w = this->findChild<QCheckBox *>("rays");
        for (auto const &object:trackedObjects) {
            lock_guard<mutex> (object.second->mux);
            object.second->rays = w->isChecked();
        }
    }

    void DarkRoom::showDistances() {
        QCheckBox *w = this->findChild<QCheckBox *>("distances");
        for (auto const &object:trackedObjects) {
            lock_guard<mutex> (object.second->mux);
            object.second->distances = w->isChecked();
        }
    }

    void DarkRoom::switch_lighthouses() {
        QCheckBox *w = this->findChild<QCheckBox *>("switch_lighthouses");
        for (auto const &object:trackedObjects) {
            lock_guard<mutex> (object.second->mux);
            object.second->switchLighthouses(w->isChecked());
        }
    }

    void DarkRoom::startCalibrateRelativeSensorDistances() {
        QCheckBox *w = this->findChild<QCheckBox *>("calibrate");
        for (uint i=0; i<trackedObjects.size(); i++) {
            lock_guard<mutex> (trackedObjects[i]->mux);
            if (w->isChecked()) {
                ROS_INFO("starting calibration thread");
                trackedObjects[i]->calibrating = true;
                trackedObjects[i]->calibrate_thread = boost::shared_ptr<boost::thread>(
                        new boost::thread(
                                [this, i](){this->trackedObjects[i]->calibrateRelativeSensorDistances();}
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

    void DarkRoom::startTriangulation() {
        QCheckBox *w = this->findChild<QCheckBox *>("triangulate");
        for (uint i=0; i<trackedObjects.size(); i++) {
            lock_guard<mutex> (trackedObjects[i]->mux);
            if (w->isChecked()) {
                ROS_INFO("starting tracking thread");
                trackedObjects[i]->tracking = true;
                trackedObjects[i]->tracking_thread = boost::shared_ptr<boost::thread>(
                        new boost::thread(
                                [this, i](){this->trackedObjects[i]->triangulateSensors();}
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

    void DarkRoom::startPoseEstimationSensorCloud() {
        QCheckBox *w = this->findChild<QCheckBox *>("poseestimation_sensor_cloud");
        for (uint i=0; i<trackedObjects.size(); i++) {
            lock_guard<mutex> (trackedObjects[i]->mux);
            if (w->isChecked()) {
                ROS_INFO("starting pose estimation thread");
                trackedObjects[i]->poseestimating = true;
                trackedObjects[i]->poseestimation_thread = boost::shared_ptr<boost::thread>(
                        new boost::thread([this, i](){
                            trackedObjects[i]->poseEstimationSensorCloud();
                        }));
                trackedObjects[i]->poseestimation_thread->detach();
            } else {
                if (trackedObjects[i]->poseestimation_thread != nullptr) {
                    trackedObjects[i]->poseestimating = false;
                    if (trackedObjects[i]->poseestimation_thread->joinable()) {
                        ROS_INFO("Waiting for pose estimation thread to terminate");
                        trackedObjects[i]->poseestimation_thread->join();
                    }
                }
            }
        }
    }

    void DarkRoom::startPoseEstimationParticleFilter() {
        QCheckBox *w = this->findChild<QCheckBox *>("particle_filter");
        for (uint i=0; i<trackedObjects.size(); i++) {
            lock_guard<mutex> (trackedObjects[i]->mux);
            if (w->isChecked()) {
                ROS_INFO("starting particle filter thread");
                trackedObjects[i]->particle_filtering = true;
                trackedObjects[i]->particlefilter_thread = boost::shared_ptr<boost::thread>(
                        new boost::thread(
                                [this, i](){this->trackedObjects[i]->poseEstimationParticleFilter();}
                        ));
                trackedObjects[i]->particlefilter_thread->detach();
            } else {
                if (trackedObjects[i]->particlefilter_thread != nullptr) {
                    trackedObjects[i]->particle_filtering = false;
                    if (trackedObjects[i]->particlefilter_thread->joinable()) {
                        ROS_INFO("Waiting for particle filter thread to terminate");
                        trackedObjects[i]->particlefilter_thread->join();
                    }
                }
            }
        }
    }

    void DarkRoom::transformPublisher() {
        ros::Rate rate(10);
        while (publish_transform) {
            tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse1, ros::Time::now(), "world", "lighthouse1"));
            tf_broadcaster.sendTransform(tf::StampedTransform(lighthouse2, ros::Time::now(), "world", "lighthouse2"));
//        tf::Matrix3x3 rot(lighthouse2.getRotation());
//
//        ROS_INFO_STREAM("rot: \n" << rot.getRow(0).getX() << "\t" << rot.getRow(0).getY() << "\t" << rot.getRow(0).getZ() << endl <<
//                                rot.getRow(1).getX() << "\t" << rot.getRow(1).getY() << "\t" << rot.getRow(1).getZ() << endl <<
//                                rot.getRow(2).getX() << "\t" << rot.getRow(2).getY() << "\t" << rot.getRow(2).getZ() << endl);
            rate.sleep();
        }
    }

    void DarkRoom::correctPose(const roboy_communication_middleware::LighthousePoseCorrection &msg) {
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

    PLUGINLIB_EXPORT_CLASS(DarkRoom, rviz::Panel)

}