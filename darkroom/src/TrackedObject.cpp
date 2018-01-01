#include "darkroom/TrackedObject.hpp"

int TrackedObject::trackeObjectInstance = 0;
bool TrackedObject::m_switch = false;

TrackedObject::TrackedObject() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "TrackedObject",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle("~"));

    darkroom_statistics_pub = nh->advertise<roboy_communication_middleware::DarkRoomStatistics>(
            "/roboy/middleware/DarkRoom/Statistics", 1);

    receiveData = true;
    sensor_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensors", 1, &TrackedObject::receiveSensorDataRoboy, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    string package_path = ros::package::getPath("darkroom");

    path = package_path+"/calibrated_objects";

    pose.setOrigin(tf::Vector3(0,0,0));
    pose.setRotation(tf::Quaternion(0,0,0,1));

    string load_yaml_command = "rosparam load "+package_path+"/params/ekf_parameters.yaml " + nh->getNamespace();
    ROS_DEBUG_STREAM("loading yaml file using this command: " << load_yaml_command);
    system(load_yaml_command.c_str());

    trackeObjectInstance++;
}

TrackedObject::~TrackedObject() {
    shutDown();
    // delete all remaining markers
    clearAll();
}

bool TrackedObject::init(const char* configFile){
    ROS_INFO_STREAM("reading config of  " << configFile);
    if(!readConfig(configFile, objectID, name, mesh, calibrated_sensors, sensors))
        return false;

    imu.setOrigin(tf::Vector3(0,0,0));
    imu.setRotation(tf::Quaternion(0,0,0,1));

    // some object specific parameters are set here:
//    nh->setParam("map_frame", "odom");
//    nh->setParam("world_frame", "world");
    nh->setParam("base_link_frame", name);
//    nh->setParam("odom_frame", "odom");
    imu_topic_name = "roboy/middleware/"+name+"/imu";
    pose_topic_name = "roboy/middleware/"+name+"/pose";

    nh->setParam("imu0", imu_topic_name);
    nh->setParam("pose0", pose_topic_name);
//    nh->setParam("publish_tf", true);
    nh->setParam("print_diagnostics", true);

//    vector<float> process_noise_covariance = {
//            0.05, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
//            0,    0.05, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0.06, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0.03, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
//            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
//            0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015
//    };
//    nh->setParam("process_noise_covariance", process_noise_covariance);

//    vector<bool> activeStatesIMU = {false, false, false, // XYZ
//                                    true, true, true, // roll, pitch, yaw
//                                    false, false, false, // d(XYZ)
//                                    false, false, false, // d(roll, pitch, yaw)
//                                    true, true, true // dd(XYZ)
//    };
//    nh->setParam("imu0_config", activeStatesIMU);
//    nh->setParam("imu0_queue_size", 100);
//    nh->setParam("imu0_differential", false);
//    nh->setParam("imu0_relative", true);
//    nh->setParam("imu0_remove_gravitational_acceleration", false);
//    nh->setParam("pose0", nh->getNamespace()+"/pose0");
//    vector<bool> activeStatesPose = {true, true, true, // XYZ
//                                true, true, true, // roll, pitch, yaw
//                                false, false, false, // d(XYZ)
//                                false, false, false, // d(roll, pitch, yaw)
//                                false, false, false // dd(XYZ)
//    };
//    nh->setParam("pose0_config", activeStatesPose);
//    nh->setParam("pose0_queue_size", 100);
//    nh->setParam("pose0_differential", false);
//    nh->setParam("pose0_relative", true);
//
    vector<float> initial_state = { 0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0
    };
    nh->setParam("initial_state", initial_state);
//    nh->setParam("publish_tf", true);
//    nh->setParam("print_diagnostics", true);
//    // start extended kalman filter
    kalman_filter_thread.reset(new boost::thread(&TrackedObject::run, this));
    return true;
}

void TrackedObject::shutDown(){
    receiveData = false;
    tracking = false;
    calibrating = false;
    poseestimating = false;
    receiveData = false;
    publish_transform = false;
    if (sensor_thread != nullptr) {
        if (sensor_thread->joinable()) {
            ROS_INFO("Waiting for sensor thread to terminate");
            sensor_thread->join();
        }
    }
    if (tracking_thread != nullptr) {
        if (tracking_thread->joinable()) {
            ROS_INFO("Waiting for tracking thread to terminate");
            tracking_thread->join();
        }
    }
    if (calibrate_thread != nullptr) {
        if (calibrate_thread->joinable()) {
            ROS_INFO("Waiting for calibration thread to terminate");
            calibrate_thread->join();
        }
    }
    if (poseestimation_thread != nullptr) {
        if (poseestimation_thread->joinable()) {
            ROS_INFO("Waiting for pose estimation thread to terminate");
            poseestimation_thread->join();
        }
    }
    if (publish_imu_transform != nullptr) {
        if (publish_imu_transform->joinable()) {
            ROS_INFO("Waiting for imu publish thread to terminate");
            publish_imu_transform->join();
        }
    }
}

void TrackedObject::connectObject(const char* broadcastIP, int port){
    uint32_t ip;
    inet_pton(AF_INET, broadcastIP, &ip);
    socket = UDPSocketPtr(new UDPSocket(port, ip));
    lock_guard<mutex> lock(mux);
    receiveData = true;
    sensor_thread = boost::shared_ptr<boost::thread>(new boost::thread(&TrackedObject::receiveSensorData, this));
    sensor_thread->detach();
}

void TrackedObject::switchLighthouses(bool switchID) {
    ROS_INFO_STREAM("switching lighthouses " << switchID);
    m_switch = switchID;
    for (auto &sensor : sensors) {
        sensor.second.switchLighthouses(switchID);
    }
}

bool TrackedObject::record(bool start) {
    if (start) {
        char str[100];
        sprintf(str, "record_%s.log", name.c_str());
        file.open(str);
        if (file.is_open()) {
            ROS_INFO("start recording");
            file << "timestamp, \tid, \tlighthouse, \trotor, \tsweepDuration[ticks]\n";
            recording = true;
            return true;
        } else {
            ROS_ERROR("could not open file");
            return false;
        }
    } else {
        ROS_INFO("saving to file recording");
        recording = false;
        file.close();
        return true;
    }
}

void TrackedObject::receiveSensorDataRoboy(const roboy_communication_middleware::DarkRoom::ConstPtr &msg) {
    if(msg->objectID != objectID) // only use messages for me
        return;
    ROS_WARN_THROTTLE(10, "receiving sensor data");
    uint id = 0;

    static int message_counter = 0;

    uint lighthouse, rotor, sensorID,sweepDuration;
    for (uint32_t const &data:msg->sensor_value) {
        lighthouse = (data >> 31) & 0x1;
        rotor = (data >> 30) & 0x1;
        int valid = (data >> 29) & 0x1;
        sweepDuration = (data & 0x1fffffff); // raw sensor duration is 50 ticks per microsecond
        sensorID= ((data >>19) & 0x3FF);
        sweepDuration= ((data & 0x7FFFF));
//        ROS_INFO_STREAM_THROTTLE(1,"timestamp:     " << timestamp << endl <<
//                "valid:         " << valid << endl <<
//                "id:            " << id << endl <<
//                "lighthouse:    " << lighthouse << endl <<
//                "rotor:         " << rotor << endl <<
//                "sweepDuration: " << sweepDuration);
        if (valid == 1) {
            if (recording) {
                lock_guard<mutex> lock(mux);
                file << msg->timestamp[sensorID] << ",\t" << sensorID << ",\t"
                     << lighthouse << ",\t" << rotor << ",\t" << sweepDuration << endl;
            }
            double angle = ticksToRadians(sweepDuration);
            sensors[sensorID].update(lighthouse, rotor, angle);
        }

        id++;
    }

    if(message_counter++%50==0){ // publish statistics from time to time
        {
            roboy_communication_middleware::DarkRoomStatistics statistics_msg;
            statistics_msg.object_name = name;
            statistics_msg.lighthouse = LIGHTHOUSE_A;
            for (uint i = 0; i < msg->sensor_value.size(); i++) {
                float horizontal, vertical;
                sensors[i].updateFrequency(LIGHTHOUSE_A, horizontal, vertical);
                statistics_msg.updateFrequency_horizontal.push_back(horizontal);
                statistics_msg.updateFrequency_vertical.push_back(vertical);
            }
            darkroom_statistics_pub.publish(statistics_msg);
        }
        {
            roboy_communication_middleware::DarkRoomStatistics statistics_msg;
            statistics_msg.object_name = name;
            statistics_msg.lighthouse = LIGHTHOUSE_B;
            for (uint i = 0; i < msg->sensor_value.size(); i++) {
                float horizontal, vertical;
                sensors[i].updateFrequency(LIGHTHOUSE_B, horizontal, vertical);
                statistics_msg.updateFrequency_horizontal.push_back(horizontal);
                statistics_msg.updateFrequency_vertical.push_back(vertical);
            }
            darkroom_statistics_pub.publish(statistics_msg);
        }
    }

}

void TrackedObject::receiveSensorData(){
    chrono::high_resolution_clock::time_point t0 = chrono::high_resolution_clock::now();
    while(receiveData){
        vector<uint32_t> id;
        vector<bool> lighthouse;
        vector<bool> rotor;
        vector<uint32_t> sweepDuration;
        if(socket->receiveSensorData(id,lighthouse,rotor,sweepDuration)){
            for(uint i=0; i<id.size(); i++) {
                ROS_INFO_STREAM_THROTTLE(10,
                        "id:              " << id[i] <<
                                            "\tlighthouse:    " << lighthouse[i] <<
                                            "\trotor:         " << rotor[i] <<
                                            "\tsweepDuration: " << sweepDuration[i]<<
                                            "\tangle: " << ticksToDegrees(sweepDuration[i]));


                sensors[id[i]].update(lighthouse[i], rotor[i], ticksToRadians(sweepDuration[i]));
                if (recording) {
                    chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
                    chrono::microseconds time_span = chrono::duration_cast<chrono::microseconds>(t1 - t0);
                    lock_guard<mutex> lock(mux);
                    file << time_span.count() << ",\t" << id[i] << ",\t"
                         << lighthouse[i] << ",\t" << rotor[i] << ",\t" << sweepDuration[i] << endl;
                }
            }
//            Vector2d angles;
//            sensors[id].get(0,angles);
//            cout << angles << endl;
//            sensors[id].get(1,angles);
//            cout << angles << endl;
        }
    }
}

void TrackedObject::publishImuFrame(){
    ros::Rate rate(30);
    while (publish_transform) {
        lock_guard<mutex> lock(mux);
        tf_broadcaster.sendTransform(tf::StampedTransform(imu, ros::Time::now(), "world", "odom"));
        rate.sleep();
    }
}