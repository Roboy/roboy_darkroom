#include "darkroom/LighthouseSimulator.hpp"

int LighthouseSimulator::class_counter = 0;

LighthouseSimulator::LighthouseSimulator(int id, const char* configFile) : id(id) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LighthouseSimulator",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    sensors_pub = nh->advertise<roboy_communication_middleware::DarkRoom>(
            "/roboy/middleware/DarkRoom/sensors", 1);
    imu_pub = nh->advertise<sensor_msgs::Imu>("/roboy/middleware/imu0", 1);

    interactive_marker_sub = nh->subscribe("/interactive_markers/feedback", 1,
                                           &LighthouseSimulator::interactiveMarkersFeedback, this);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

//    string package_path = ros::package::getPath("darkroom");
//    string path = package_path + "/calibrated_objects";
    ROS_INFO_STREAM("reading config of " << configFile);
    readConfig(configFile, objectID, name, mesh, calibrated_sensors, sensors);

    for (auto &sensor:sensors) {
        Vector3d rel_location;
        sensor.second.getRelativeLocation(rel_location);
        sensor_position[sensor.first] << rel_location[0], rel_location[1], rel_location[2], 1;
    }

//    char str[100];
//    sprintf(str, "%s_simulated_%d", name.c_str(), id);

//    name = string(str);

    class_counter++;

    tf::Vector3 origin(0, 1, 0);
    make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, origin,
                   true, 0.1, (id == 0 ? "lighthouse1" : "lighthouse2"), name.c_str(), "");
    relative_object_pose.setOrigin(origin);
    relative_object_pose.setRotation(tf::Quaternion(0, 0, 1, 0));

}

LighthouseSimulator::~LighthouseSimulator() {
    {
        lock_guard<mutex> lock(mux);
        sensor_publishing = false;
        imu_publishing = false;
    }
}

void LighthouseSimulator::PublishSensorData() {
    ros::Duration d(1);
    d.sleep();

    ros::Rate rate(120);
    bool angle_switch = false;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (sensor_publishing) {
        Matrix4d RT_object2lighthouse;
        if (!getTransform(name.c_str(), (id == 0 ? "lighthouse1" : "lighthouse2"), RT_object2lighthouse))
            continue;

        Vector4d lighthouse_pos(0, 0, 0, 1);
        roboy_communication_middleware::DarkRoom msg;

        for (auto const &sensor:sensor_position) {
            Vector4d sensor_pos;
            sensor_pos = RT_object2lighthouse * sensor.second;
            double distance = sqrt(pow(sensor_pos[0], 2.0) + pow(sensor_pos[1], 2.0) + pow(sensor_pos[2], 2.0));
            double elevation = 180.0 - acos(sensor_pos[2] / distance) * 180.0 / M_PI;
            double azimuth = atan2(sensor_pos[1], sensor_pos[0]) * 180.0 / M_PI;

            uint32_t sensor_value;
            if (elevation >= 0 && elevation <= 180.0 && azimuth >= 0 &&
                azimuth <= 180.0) { // if the sensor is visible by lighthouse

                if (!angle_switch) {
                    uint32_t elevation_in_ticks = (uint32_t) (degreesToTicks(elevation));
                    sensor_value = (uint32_t) (id << 31 | 1 << 30 | true << 29 | sensor.first<<19 | elevation_in_ticks & 0x7FFFF);
                    if (sensor.first == 0)
                        ROS_DEBUG_THROTTLE(1, "elevation: %lf in ticks: %d", elevation, elevation_in_ticks);
                } else {
                    uint32_t azimuth_in_ticks = (uint32_t) (degreesToTicks(azimuth));
                    sensor_value = (uint32_t) (id << 31 | 0 << 30 | true << 29 | sensor.first<<19 | azimuth_in_ticks & 0x7FFFF);
                    if (sensor.first == 0)
                        ROS_DEBUG_THROTTLE(1, "azimuth: %lf in ticks: %d", azimuth, azimuth_in_ticks);
                }
                Vector3d pos(sensor_pos[0], sensor_pos[1], sensor_pos[2]);
                publishSphere(pos, (id == 0 ? "lighthouse1" : "lighthouse2"), "simulated_sensor_positions",
                              sensor.first + id * sensor_position.size() + 6543, COLOR(0, 1, 0, 1), 0.01);
            } else {
                Vector3d pos(sensor_pos[0], sensor_pos[1], sensor_pos[2]);
                publishSphere(pos, (id == 0 ? "lighthouse1" : "lighthouse2"), "simulated_sensor_positions",
                              sensor.first + id * sensor_position.size() + 7543, COLOR(1, 0, 0, 1), 0.01, 1);
            }
            high_resolution_clock::time_point t1 = high_resolution_clock::now();
            microseconds time_span = duration_cast<microseconds>(t1 - t0);
            msg.timestamp.push_back(time_span.count());
            msg.sensor_value.push_back(sensor_value);
//            Vector3d origin(0,0,0);
//            Vector3d dir(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
//            publishRay(origin, dir,(id==0?"lighthouse1":"lighthouse2"),"simulated_sensor_rays",
//                       sensor.first+class_counter*sensor_position.size()+8543, COLOR(0,0,0,1), 0.01);
        }
        angle_switch = !angle_switch;

        if (!msg.sensor_value.empty())
            sensors_pub.publish(msg);

        rate.sleep();
    }
}

void LighthouseSimulator::PublishImuData() {
    ros::Duration d(3);
    d.sleep();
    ros::Rate rate(1);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (imu_publishing) {
        Matrix4d RT_object2world;
        if (!getTransform(name.c_str(), "world", RT_object2world))
            continue;

        sensor_msgs::Imu msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "calibrationCube";
        msg.orientation.w = 1;
        msg.linear_acceleration_covariance = {
                1, 0, 0,
                0, 1, 0,
                0, 0, 1
        };

        msg.linear_acceleration.x = 0 + rand()/(double)RAND_MAX * IMU_ACC_NOISE;
        msg.linear_acceleration.y = 0 + rand()/(double)RAND_MAX * IMU_ACC_NOISE;
        msg.linear_acceleration.z = 9.81 + rand()/(double)RAND_MAX * IMU_ACC_NOISE;

        imu_pub.publish(msg);
        rate.sleep();
    }
}

void LighthouseSimulator::interactiveMarkersFeedback(const visualization_msgs::InteractiveMarkerFeedback &msg) {
    tf::Vector3 position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    tf::Quaternion orientation(msg.pose.orientation.x, msg.pose.orientation.y,
                               msg.pose.orientation.z, msg.pose.orientation.w);
    if (strcmp(msg.marker_name.c_str(), name.c_str()) == 0) {
        relative_object_pose.setOrigin(position);
        relative_object_pose.setRotation(orientation);
    }
}