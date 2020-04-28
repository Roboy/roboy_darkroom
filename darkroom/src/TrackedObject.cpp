#include "darkroom/TrackedObject.hpp"

int LighthouseEstimator::trackedObjectInstance = 0;
bool TrackedObject::m_switch = false;

TrackedObject::TrackedObject(rclcpp::Node::SharedPtr nh):robot_localization::RosEkf(nh->get_node_options()), nh(nh) {

    clock = nh->get_clock();
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);
    parameters_client = std::make_shared<rclcpp::SyncParametersClient>(nh);

    while (!parameters_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(nh->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(nh->get_logger(), "parameter sync service not available, waiting again...");
    }

    darkroom_statistics_pub = nh->create_publisher<roboy_middleware_msgs::msg::DarkRoomStatistics>(
            "/roboy/middleware/DarkRoom/Statistics", 1);

    receiveData = true;
    sensor_sub = nh->create_subscription<roboy_middleware_msgs::msg::DarkRoomSensor>("/roboy/middleware/DarkRoom/sensors",1,bind(&TrackedObject::receiveSensorDataRoboy, this, placeholders::_1));
//    sensor_sub =

//    nh->create_subscription("/roboy/middleware/DarkRoom/sensors", 1, bind(&TrackedObject::receiveSensorDataRoboy, this, placeholders::_1));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(nh);
    executor.spin();
//    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(2));
//    spinner->start();

    string package_path = "";//ros::package::getPath("darkroom");
    //TODO relocate lighthouse_calibration.yaml
    string calibration_path = package_path + "/params/lighthouse_calibration.yaml";
    readCalibrationConfig(calibration_path,LIGHTHOUSE_A,calibration[LIGHTHOUSE_A]);
    readCalibrationConfig(calibration_path,LIGHTHOUSE_B,calibration[LIGHTHOUSE_B]);


    path = package_path+"/calibrated_objects";

    pose.setOrigin(tf2::Vector3(0,0,0));
    pose.setRotation(tf2::Quaternion(0,0,0,1));

    setParametersFromYaml(nh, "ekf_parameters.yaml");
    //TODO fix path
//    string load_yaml_command = "rosparam load "+package_path+"/params/ekf_parameters.yaml " + nh->getNamespace();
//    ROS_DEBUG_STREAM("loading yaml file using this command: " << load_yaml_command);
//    system(load_yaml_command.c_str());

    trackedObjectInstance++;
}

TrackedObject::~TrackedObject() {
    shutDown();
    // delete all remaining markers
    clearAll();
}

bool TrackedObject::init(const char* configFile){
    RCLCPP_INFO_STREAM(nh->get_logger(),"reading config of  " << configFile);
    if(!readConfig(configFile, objectID, name, mesh, calibrated_sensors, sensors, calibration_angles))
        return false;

    for(auto &sensor:sensors){
        Vector3d rel_pos;
        sensor.second.getRelativeLocation(rel_pos);
        publishSphere(rel_pos,"world","relative_locations",rand(),COLOR(0,1,0,1),0.01,10);
    }

    imu.setOrigin(tf2::Vector3(0,0,0));
    imu.setRotation(tf2::Quaternion(0,0,0,1));

    vector<float> initial_state = { 0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0,
                                    0.0,  0.0,  0.0
    };

    auto set_parameters_results = parameters_client->set_parameters(
            {
                rclcpp::Parameter("base_link_frame", name),
                rclcpp::Parameter("imu0", imu_topic_name),
                rclcpp::Parameter("pose0", pose_topic_name),
                rclcpp::Parameter("print_diagnostics", true),
                rclcpp::Parameter("initial_state", initial_state)

            }
    );
    for (auto & result : set_parameters_results) {
        if (!result.successful) {
            RCLCPP_ERROR(nh->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
        }
    }
    // some object specific parameters are set here:
//    nh->setParam("map_frame", "odom");
//    nh->setParam("world_frame", "world");
    //nh->setParam("base_link_frame", name);
//    nh->setParam("odom_frame", "odom");
//    std::replace(name.begin(),name.end(),"-","_");
    imu_topic_name = "roboy/middleware/"+name+"/imu";
    pose_topic_name = "roboy/middleware/"+name+"/pose";

    //nh->setParam("imu0", imu_topic_name);
    //nh->setParam("pose0", pose_topic_name);
//    nh->setParam("publish_tf", true);
    //nh->setParam("print_diagnostics", true);

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

//    nh->setParam("publish_tf", true);
//    nh->setParam("print_diagnostics", true);
//    // start extended kalman filter
//    kalman_filter_thread.reset(new boost::thread(&TrackedObject::run, this));
    return true;
}

void TrackedObject::shutDown(){
    mux.lock();
    receiveData = false;
    tracking = false;
    calibrating = false;
    poseestimating = false;
    poseestimating_epnp = false;
    poseestimating_multiLighthouse = false;
    receiveData = false;
    publish_transform = false;
    rays = false;
    if (sensor_thread != nullptr) {
        if (sensor_thread->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for sensor thread to terminate");
            sensor_thread->join();
        }
    }
    if (tracking_thread != nullptr) {
        if (tracking_thread->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for tracking thread to terminate");
            tracking_thread->join();
        }
    }
    if (rays_thread != nullptr) {
        if (rays_thread->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for rays thread to terminate");
            rays_thread->join();
        }
    }
    if (calibrate_thread != nullptr) {
        if (calibrate_thread->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for calibration thread to terminate");
            calibrate_thread->join();
        }
    }
    if (poseestimation_thread != nullptr) {
        if (poseestimation_thread->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for pose estimation thread to terminate");
            poseestimation_thread->join();
        }
    }
    if (relative_pose_epnp_thread != nullptr) {
        if (relative_pose_epnp_thread->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for pose estimation epnp thread to terminate");
            relative_pose_epnp_thread->join();
        }
    }
    if (object_pose_estimation_multi_lighthouse_thread != nullptr) {
        if (object_pose_estimation_multi_lighthouse_thread->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for pose estimation multi lighthouse thread to terminate");
            object_pose_estimation_multi_lighthouse_thread->join();
        }
    }
    if (publish_imu_transform != nullptr) {
        if (publish_imu_transform->joinable()) {
            RCLCPP_INFO(nh->get_logger(),"Waiting for imu publish thread to terminate");
            publish_imu_transform->join();
        }
    }
    mux.unlock();
}

void TrackedObject::connectObject(const char* broadcastIP, int port){
    uint32_t ip;
    inet_pton(AF_INET, broadcastIP, &ip);
    socket = UDPSocketPtr(new UDPSocket(port, ip, false));
    mux.lock();
    receiveData = true;
    sensor_thread = boost::shared_ptr<boost::thread>(new boost::thread(&TrackedObject::receiveSensorData, this));
    sensor_thread->detach();
    mux.unlock();
}

void TrackedObject::switchLighthouses(bool switchID) {
    RCLCPP_INFO_STREAM(nh->get_logger(),"switching lighthouses " << switchID);
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
            RCLCPP_INFO(nh->get_logger(),"start recording");
            file << "timestamp, \tid, \tlighthouse, \trotor, \tsweepDuration[ticks], \tangle[rad]\n";
            recording = true;
            return true;
        } else {
            RCLCPP_ERROR(nh->get_logger(),"could not open file");
            return false;
        }
    } else {
        RCLCPP_INFO(nh->get_logger(),"saving to file recording");
        recording = false;
        file.close();
        return true;
    }
}

void TrackedObject::receiveSensorDataRoboy(const roboy_middleware_msgs::msg::DarkRoom::SharedPtr msg) {
    if(msg->object_id != objectID){
        // only use messages for me
        rclcpp::Clock steady_clock(RCL_STEADY_TIME);
        RCLCPP_DEBUG_STREAM_THROTTLE(nh->get_logger(),steady_clock, 1,"receiving sensor data, but it's not for me " << objectID << " " << msg->object_id);
        return;
    }
    rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(nh->get_logger(), steady_clock,10, "receiving sensor data");
    uint id = 0;

    static int message_counter = 0;

    uint lighthouse, rotor, sensorID,sweepDuration;

    for (uint32_t const &data:msg->sensor_value) {
        lighthouse = (data >> 31) & 0x1;
        rotor = (data >> 30) & 0x1;
        int valid = (data >> 29) & 0x1;
        sensorID= ((data >>19) & 0x3FF);
        sweepDuration= ((data & 0x7FFFF));
//        ROS_INFO_STREAM_THROTTLE(1,
//                "valid:         " << valid << endl <<
//                "sensorID:      " << sensorID << endl <<
//                "lighthouse:    " << lighthouse << endl <<
//                "rotor:         " << rotor << endl <<
//                "sweepDuration: " << sweepDuration << endl <<
//                "angle:         " << ticksToRadians(sweepDuration));
        if (valid == 1) {
            double angle = ticksToRadians(sweepDuration);
            sensors[sensorID].update(lighthouse, rotor, angle);
            if (recording) {
                mux.lock();
                file << msg->timestamp[sensorID] << ",\t" << sensorID << ",\t"
                     << lighthouse << ",\t" << rotor << ",\t" << sweepDuration << ",\t" << angle << endl;
                mux.unlock();
            }
        }

        id++;
    }

    if(message_counter++%50==0){ // publish statistics from time to time
        {
            roboy_middleware_msgs::msg::DarkRoomStatistics statistics_msg;
            statistics_msg.object_name = name;
            statistics_msg.lighthouse = LIGHTHOUSE_A;
            for (uint i = 0; i < msg->sensor_value.size(); i++) {
                float horizontal, vertical;
                sensors[i].updateFrequency(LIGHTHOUSE_A, horizontal, vertical);
                statistics_msg.update_frequency_horizontal.push_back(horizontal);
                statistics_msg.update_frequency_vertical.push_back(vertical);
            }
            darkroom_statistics_pub->publish(statistics_msg);
        }
        {
            roboy_middleware_msgs::msg::DarkRoomStatistics statistics_msg;
            statistics_msg.object_name = name;
            statistics_msg.lighthouse = LIGHTHOUSE_B;
            for (uint i = 0; i < msg->sensor_value.size(); i++) {
                float horizontal, vertical;
                sensors[i].updateFrequency(LIGHTHOUSE_B, horizontal, vertical);
                statistics_msg.update_frequency_horizontal.push_back(horizontal);
                statistics_msg.update_frequency_vertical.push_back(vertical);
            }
            darkroom_statistics_pub->publish(statistics_msg);
        }
    }

}

void TrackedObject::receiveSensorData(){
    chrono::high_resolution_clock::time_point t0 = chrono::high_resolution_clock::now();
    int message_counter = 0;
    while(receiveData){
        vector<uint32_t> id;
        vector<bool> lighthouse;
        vector<bool> rotor;
        vector<uint32_t> sweepDuration;
        if(socket->receiveSensorData(id,lighthouse,rotor,sweepDuration)){
            for(uint i=0; i<id.size(); i++) {
                double angle = ticksToRadians(sweepDuration[i]);
                //TODO deal with throttle and clock
                //RCLCPP_INFO_THROTTLE(nh->get_logger(), clock, 10, "id: %d lighthouse: %d rotor: %d sweepDuration: %d angle: %.3f", id[i], lighthouse[i],
                //          rotor[i], sweepDuration[i],ticksToDegrees(sweepDuration[i]));
                sensors[id[i]].update(lighthouse[i], rotor[i], angle);
                if (recording) {
                    chrono::high_resolution_clock::time_point t1 = chrono::high_resolution_clock::now();
                    chrono::microseconds time_span = chrono::duration_cast<chrono::microseconds>(t1 - t0);
                    mux.lock();
                    file << time_span.count() << ",\t" << id[i] << ",\t"
                         << lighthouse[i] << ",\t" << rotor[i] << ",\t" << sweepDuration[i] << ",\t" << angle << endl;
                    mux.unlock();
                }
            }

            if(message_counter++%50==0){ // publish statistics from time to time
                {
                    roboy_middleware_msgs::msg::DarkRoomStatistics statistics_msg;
                    statistics_msg.object_name = name;
                    statistics_msg.lighthouse = LIGHTHOUSE_A;
                    for (uint32_t i:id) {
                        float horizontal, vertical;
                        sensors[i].updateFrequency(LIGHTHOUSE_A, horizontal, vertical);
                        statistics_msg.update_frequency_horizontal.push_back(horizontal);
                        statistics_msg.update_frequency_vertical.push_back(vertical);
                    }
                    darkroom_statistics_pub->publish(statistics_msg);
                }
                {
                    roboy_middleware_msgs::msg::DarkRoomStatistics statistics_msg;
                    statistics_msg.object_name = name;
                    statistics_msg.lighthouse = LIGHTHOUSE_B;
                    for (uint32_t i:id) {
                        float horizontal, vertical;
                        sensors[i].updateFrequency(LIGHTHOUSE_B, horizontal, vertical);
                        statistics_msg.update_frequency_horizontal.push_back(horizontal);
                        statistics_msg.update_frequency_vertical.push_back(vertical);
                    }
                    darkroom_statistics_pub->publish(statistics_msg);
                }
            }
        }
    }
}

void TrackedObject::publishImuFrame(){
    rclcpp::Rate rate(30);
    while (publish_transform) {
        geometry_msgs::msg::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.frame_id = "world";
        tmp_tf_stamped.child_frame_id = "odom";
        //TODO set correct timestamp
        //tmp_tf_stamped.header.stamp = tf2::get_now(); 
//        lock_guard<mutex> lock(mux);
        tf_broadcaster->sendTransform(tmp_tf_stamped);
        rate.sleep();
    }
}