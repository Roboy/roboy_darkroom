#include <darkroom/LighthouseSimulator.hpp>

int LighthouseSimulator::class_counter = 0;

LighthouseSimulator::LighthouseSimulator(int id, fs::path configFile) : id(id) {
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

    if(!readConfig(configFile, objectID, name, meshPath, calibrated_sensors, sensors))
        return;

    ROS_DEBUG_STREAM("loading mesh file " << meshPath);
    pcl::PolygonMesh m;
    pcl::io::loadPolygonFile(meshPath.c_str(),m);
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(m.cloud, vertices);
    mesh.polygons = m.polygons;
    for(int i=0;i<vertices.size();i++){
        // convert from mm to meter
        mesh.vertices.push_back(Vector4d(vertices[i].x*0.001, vertices[i].y*0.001, vertices[i].z*0.001, 1));
    }
    ROS_INFO("done loading mesh %s: %ld triangles, %ld vertices", meshPath.filename().c_str(),
             mesh.polygons.size(), mesh.vertices.size());

    for (auto &sensor:sensors) {
        Vector3d rel_location;
        sensor.second.getRelativeLocation(rel_location);
        sensor_position[sensor.first] << rel_location[0], rel_location[1], rel_location[2], 1;
    }

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
    Matrix4d RT_object2lighthouse, RT_object2lighthouse_new;
    vector<Vector4d> vertices = mesh.vertices;
    bool pose_changed = true;
    vector<bool> sensor_visible(sensor_position.size(), false);
    while (sensor_publishing) {
        if (!getTransform(name.c_str(), (id == 0 ? "lighthouse1" : "lighthouse2"), RT_object2lighthouse_new))
            continue;

        if(!RT_object2lighthouse_new.isApprox(RT_object2lighthouse)){
            pose_changed = true;
            vertices = mesh.vertices;
            // apply transform to all vertices
            for(auto &v:vertices){
                v = RT_object2lighthouse_new*v;
            }
        }else{
            pose_changed = false;
        }

        RT_object2lighthouse = RT_object2lighthouse_new;
        Vector4d lighthouse_pos(0, 0, 0, 1);
        roboy_communication_middleware::DarkRoom msg;

        int i = 0;
        for (auto const &sensor:sensor_position) {
            Vector4d sensor_pos;
            sensor_pos = RT_object2lighthouse * sensor.second;
            double distance = sqrt(pow(sensor_pos[0], 2.0) + pow(sensor_pos[1], 2.0) + pow(sensor_pos[2], 2.0));
            double elevation = 180.0 - acos(sensor_pos[2] / distance) * 180.0 / M_PI;
            double azimuth = atan2(sensor_pos[1], sensor_pos[0]) * 180.0 / M_PI;

            if(pose_changed){ // we gotta check if the sensor is visible
                Vector3d ray(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
                sensor_visible[i] = checkIfSensorVisible(vertices,ray);
            }

            uint32_t sensor_value;
            if (elevation >= 0 && elevation <= 180.0 && azimuth >= 0 &&
                azimuth <= 180.0 && sensor_visible[i]) { // if the sensor is visible by lighthouse

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
            i++;
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

bool LighthouseSimulator::checkIfSensorVisible(vector<Vector4d> &vertices, Vector3d &ray){
    // the ray origin is the lighthouse origin
    Vector3d origin(0,0,0);
    int triangle_intersections = 0;
    int messageID = 6000+1000*id;
    for(auto polygon:mesh.polygons){
        double u, v, t;
        if(rayIntersectsTriangle(origin, ray, vertices[polygon.vertices[0]], vertices[polygon.vertices[1]],
                                 vertices[polygon.vertices[2]],u,v,t)){
            triangle_intersections++;
//            Vector3d intersection = t*ray;
//            publishRay(origin,intersection ,(id == 0 ? "lighthouse1" : "lighthouse2"),"vertices", messageID++, COLOR(0,0,1,1));
        }
//        Vector3d v0(vertices[polygon.vertices[0]][0], vertices[polygon.vertices[0]][1], vertices[polygon.vertices[0]][2]);
//        publishSphere(v0,(id == 0 ? "lighthouse1" : "lighthouse2"),"vertices", messageID++, COLOR(0,0,1,1));
    }
    ROS_INFO_THROTTLE(1,"triangle intersections %d", triangle_intersections);
    // if there is none or only one triangle intersection, the sensor is visible
    if(triangle_intersections<2)
        return true;
    else
        return false;
}

bool LighthouseSimulator::rayIntersectsTriangle(Vector3d &origin, Vector3d &ray,
                                                Vector4d &v0, Vector4d &v1, Vector4d &v2,
                                                double &u, double &v, double &t){
    Vector3d e1(v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]);
    Vector3d e2(v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]);

    Vector3d h = ray.cross(e2);
    double a = e1.dot(h);

    if (a > -0.00001 && a < 0.00001)
        return false;

    double f = 1.0/a;
    Vector3d s(origin[0]-v0[0], origin[1]-v0[1], origin[2]-v0[2]);
    u = f * s.dot(h);

    if (u < 0.0 || u > 1.0)
        return false;

    Vector3d q = s.cross(e1);
    v = f * ray.dot(q);

    if (v < 0.0 || u + v > 1.0)
        return false;

    // at this stage we can compute t to find out where
    // the intersection point is on the line
    t = f * e2.dot(q);

    if (t > 0.00001) // ray intersection
        return true;
    else // this means that there is a line intersection
        // but not a ray intersection
        return false;
}