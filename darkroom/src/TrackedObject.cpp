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
    nh = ros::NodeHandlePtr(new ros::NodeHandle);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    if (const char *env_p = getenv("DARKROOM_CALIBRATED_OBJECTS")) {
        path = env_p;
        ROS_INFO_STREAM("using DARKROOM_CALIBRATED_OBJECTS: " << path);
        readConfig(path + "/protoType3.yaml");
    } else
        ROS_WARN("could not get DARKROOM_CALIBRATED_OBJECTS environmental variable");

    trackeObjectInstance++;
}

TrackedObject::~TrackedObject() {
    receiveData = false;
    tracking = false;
    calibrating = false;
    poseestimating = false;
    receiveData = false;
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
}

void TrackedObject::connectRoboy() {
    receiveData = true;
    sensor_sub = nh->subscribe("/roboy/middleware/DarkRoom/sensors", 1, &TrackedObject::receiveSensorDataRoboy, this);
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
        file.open("record.log");
        if (file.is_open()) {
            ROS_INFO("start recording");
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

bool TrackedObject::readConfig(string filepath) {
    YAML::Node config = YAML::LoadFile(filepath);
    objectID = config["ObjectID"].as<int>();
    name = config["name"].as<string>();
    mesh = config["mesh"].as<string>();
    vector<vector<float>> relative_locations =
            config["sensor_relative_locations"].as<vector<vector<float >>>();
    sensors.clear();
    calibrated_sensors.clear();
    cout << "using calibrated sensors: ";
    for (int i = 0; i < relative_locations.size(); i++) {
        Vector3d relLocation(relative_locations[i][1], relative_locations[i][2], relative_locations[i][3]);
        sensors[relative_locations[i][0]].setRelativeLocation(relLocation);
        calibrated_sensors.push_back((int) relative_locations[i][0]);
        cout << "\t" << calibrated_sensors.back();
    }
    cout << endl;

    return true;
}

void TrackedObject::receiveSensorDataRoboy(const roboy_communication_middleware::DarkRoom::ConstPtr &msg) {
    ROS_WARN_THROTTLE(5, "receiving sensor data");
    unsigned short timestamp = (unsigned short) (ros::Time::now().sec & 0xFF);
    uint id = 0;
    for (uint32_t const &data:msg->sensor_value) {
        uint lighthouse, rotor, sweepDuration;
        lighthouse = (data >> 31) & 0x1;
        rotor = (data >> 30) & 0x1;
        int valid = (data >> 29) & 0x1;
        sweepDuration = (data & 0x1fffffff); // raw sensor duration is 50 ticks per microsecond
//        ROS_INFO_STREAM_THROTTLE(1,"timestamp:     " << timestamp << endl <<
//                "valid:         " << valid << endl <<
//                "id:            " << id << endl <<
//                "lighthouse:    " << lighthouse << endl <<
//                "rotor:         " << rotor << endl <<
//                "sweepDuration: " << sweepDuration);
        if (valid == 1) {
            if (recording) {
                file << "\n---------------------------------------------\n"
                     << "timestamp:     " << timestamp << endl
                     << "id:            " << id << endl
                     << "lighthouse:    " << lighthouse << endl
                     << "rotor:         " << rotor << endl
                     << "sweepDuration: " << sweepDuration << endl;
            }

            sensors[id].update(lighthouse, rotor, timestamp, ticksToRadians(sweepDuration));
        }
        id++;
    }
}

void TrackedObject::receiveSensorData(){
    while(receiveData){
        uint32_t id;
        bool lighthouse;
        bool rotor;
        uint16_t sweepDuration;
        if(socket->receiveSensorData(id,lighthouse,rotor,sweepDuration)){
            ROS_INFO_STREAM(
                "id:              " << id <<
                "\tlighthouse:    " << lighthouse <<
                "\trotor:         " << rotor <<
                "\tsweepDuration: " << sweepDuration);
            unsigned short timestamp = (unsigned short) (ros::Time::now().sec & 0xFF);
            sensors[id].update(lighthouse, rotor, timestamp, uSecsToRadians(sweepDuration));
//            Vector2d angles;
//            sensors[id].get(0,angles);
//            cout << angles << endl;
//            sensors[id].get(1,angles);
//            cout << angles << endl;
        }
    }
}

bool TrackedObject::writeConfig(string filepath) {
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }

    YAML::Node config;
    config["ObjectID"] = objectID;
    config["name"] = name;
    config["mesh"] = mesh;
    for (auto &sensor : sensors) {
        if (!sensor.second.sensorCalibrated())
            continue;
        YAML::Node node = YAML::Load("[0, 0, 0, 0]");
        Vector3d relative_location;
        sensor.second.getRelativeLocation(relative_location);
        // first number is the sensor id
        node[0] = sensor.first;
        for (int i = 1; i <= 3; i++) {
            node[i] = relative_location(i - 1);
        }
        config["sensor_relative_locations"].push_back(node);
    }

    fout << config;
    return true;
}
