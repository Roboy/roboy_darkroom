#include <darkroom/LighthouseSimulator.hpp>

LighthouseSimulator::LighthouseSimulator(int id, vector<fs::path> &configFile) : id(id) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LighthouseSimulator",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    sensors_pub = nh->advertise<roboy_communication_middleware::DarkRoom>(
            "/roboy/middleware/DarkRoom/sensors", 100);

    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    name.resize(configFile.size());
    meshes.resize(configFile.size());
    sensor_position.resize(configFile.size());
    sensor_visible.resize(configFile.size());
    objectID.resize(configFile.size());

    for(int i=0;i<configFile.size();i++){
        fs::path meshPath;
        vector<int> calibrated_sensors;
        map<int, Sensor> sensors;
        map<int,vector<double>> calibrationAngles;

        if(!readConfig(configFile[i], objectID[i], name[i], meshPath, calibrated_sensors, sensors, calibrationAngles))
            return;
        imu_pub.push_back(nh->advertise<sensor_msgs::Imu>(("/roboy/middleware/"+name[i]+"/imu").c_str(), 1));

        if(!meshPath.empty()){
            ROS_DEBUG_STREAM("loading mesh file " << meshPath);
            pcl::PolygonMesh m;
            pcl::io::loadPolygonFile(meshPath.c_str(),m);
            pcl::PointCloud<pcl::PointXYZ> vertices;
            pcl::fromPCLPointCloud2(m.cloud, vertices);
            meshes[i].polygons = m.polygons;
            for(int j=0;j<vertices.size();j++){
                // convert from mm to meter
                meshes[i].vertices.push_back(Vector4d(vertices.at(j).x*0.001, vertices.at(j).y*0.001, vertices.at(j).z*0.001, 1));
            }
            meshes[i].vertices_transformed = meshes[i].vertices;
            ROS_INFO("done loading mesh %s: %ld triangles, %ld vertices", meshPath.filename().c_str(),
                     meshes[i].polygons.size(), meshes[i].vertices.size());
            has_mesh = true;
        }

        for (auto &sensor:sensors) {
            Vector3d rel_location;
            sensor.second.getRelativeLocation(rel_location);
            sensor_position[i][sensor.first] << rel_location[0], rel_location[1], rel_location[2], 1;
            sensor_visible[i].push_back(true);
        }
    }
}

LighthouseSimulator::~LighthouseSimulator() {
    lock_guard<mutex> lock(mux);
    sensor_publishing = false;
    imu_publishing = false;

    if(sensor_thread!=nullptr){
        if(sensor_thread->joinable()){
            ROS_INFO("waiting for sensor thread to shutdown");
            sensor_thread->join();
        }
    }

    if(imu_thread!=nullptr){
        if(imu_thread->joinable()){
            ROS_INFO("waiting for imu thread to shutdown");
            imu_thread->join();
        }
    }
}

void LighthouseSimulator::PublishSensorData() {
    ros::Rate rate(120);
    bool motor = HORIZONTAL;
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    vector<Matrix4d> RT_object2lighthouse(meshes.size()), RT_object2lighthouse_new(meshes.size());
    vector<bool> pose_changed(meshes.size(),true);

    while (sensor_publishing) {
        for (int i=0; i<meshes.size();i++) {
            // we need the transform from object to lighthouse frame
            if (!getTransform(name[i].c_str(), (id == 0 ? "lighthouse1" : "lighthouse2"), RT_object2lighthouse_new[i]))
                continue;

            // because checking the visiblity of a sensor is costly, we only do it, when the pose has changed
            if(!RT_object2lighthouse_new[i].isApprox(RT_object2lighthouse[i])){
                // only check visibility if this parameter is defined and only if we have a mesh obviously
                if(nh->hasParam("check_visibility") && has_mesh) {
                    pose_changed[i] = true;
                    // apply transform to all vertices
                    for (int k = 0; k < meshes[i].vertices.size(); k++) {
                        meshes[i].vertices_transformed[k] = RT_object2lighthouse_new[i] * meshes[i].vertices[k];
                    }
                }
                RT_object2lighthouse[i] = RT_object2lighthouse_new[i];
            }else{
                pose_changed[i] = false;
            }

            roboy_communication_middleware::DarkRoom msg;

            msg.objectID = objectID[i];
            int j= 0;
            for (auto const &sensor:sensor_position[i]) {
                Vector4d sensor_pos;
                sensor_pos = RT_object2lighthouse[i] * sensor.second;

                Matrix4d tilt_trafo = Matrix4d::Identity();
                tilt_trafo.block(0, 0, 3, 3) << cos(calibration[id][motor].tilt), 0, sin(calibration[id][motor].tilt),
                        0, 1, 0,
                        -sin(calibration[id][motor].tilt), 0, cos(calibration[id][motor].tilt);

                Vector4d sensor_pos_motor_vertical, sensor_pos_motor_horizontal;
                sensor_pos_motor_vertical = sensor_pos - Vector4d(-AXIS_OFFSET,0,0,0);
                sensor_pos_motor_horizontal = sensor_pos - Vector4d(0,0,-AXIS_OFFSET,0);

                double elevation = M_PI -  atan2(sensor_pos_motor_vertical(1), sensor_pos_motor_vertical(2));
                double azimuth = atan2(sensor_pos_motor_horizontal[1], sensor_pos_motor_horizontal[0]);

                ROS_DEBUG_STREAM_THROTTLE(1,"measured sensor pos: " << sensor_pos.transpose() << "\t elevation " << elevation << "\t azimuth " <<azimuth);

                // excentric parameters, assumed to be from y axis -> cos
                double temp_elevation1 = calibration[id][VERTICAL].curve*pow(cos(azimuth)*sin(elevation),2.0);
                double temp_elevation2 = calibration[id][VERTICAL].gibmag*cos(elevation+calibration[id][VERTICAL].gibphase);
                double temp_azimuth1 = calibration[id][HORIZONTAL].curve*pow(-sin(azimuth)*cos(elevation),2.0);
                double temp_azimuth2 = calibration[id][HORIZONTAL].gibmag*cos(azimuth+calibration[id][HORIZONTAL].gibphase);
                elevation += calibration[id][VERTICAL].phase + temp_elevation1 + temp_elevation2;
                azimuth += calibration[id][HORIZONTAL].phase + temp_azimuth1 + temp_azimuth2;

                uint32_t sensor_value;
                if (elevation >= 0 && elevation <= 180.0 && azimuth >= 0 &&
                    azimuth <= 180.0 && sensor_visible[i][j]) { // if the sensor is visible by lighthouse

                    if (motor == VERTICAL) {
                        uint32_t elevation_in_ticks = (uint32_t) (radiansToTicks(elevation));
                        sensor_value = (uint32_t) (id << 31 | 1 << 30 | true << 29 | sensor.first<<19 | elevation_in_ticks & 0x7FFFF);
                        if (sensor.first == 0)
                            ROS_DEBUG_THROTTLE(1, "elevation: %lf in ticks: %d", elevation, elevation_in_ticks);
                    } else if(motor == HORIZONTAL) {
                        uint32_t azimuth_in_ticks = (uint32_t) (radiansToTicks(azimuth));
                        sensor_value = (uint32_t) (id << 31 | 0 << 30 | true << 29 | sensor.first<<19 | azimuth_in_ticks & 0x7FFFF);
                        if (sensor.first == 0)
                            ROS_DEBUG_THROTTLE(1, "azimuth: %lf in ticks: %d", azimuth, azimuth_in_ticks);
                    }
                    Vector3d pos(sensor_pos[0], sensor_pos[1], sensor_pos[2]);
                    publishSphere(pos, (id == 0 ? "lighthouse1" : "lighthouse2"), "simulated_sensor_positions",
                                  sensor.first + id * sensor_position.size() + i*1000000, COLOR(0.5, 0, 0.5, 1), 0.01);
                } else {
                    Vector3d pos(sensor_pos[0], sensor_pos[1], sensor_pos[2]);
                    publishSphere(pos, (id == 0 ? "lighthouse1" : "lighthouse2"), "simulated_sensor_positions",
                                  sensor.first + id * sensor_position.size() + i*1000000, COLOR(1, 0, 0, 1), 0.01, 1);
                }
                high_resolution_clock::time_point t1 = high_resolution_clock::now();
                microseconds time_span = duration_cast<microseconds>(t1 - t0);
                msg.timestamp.push_back(time_span.count());
                msg.sensor_value.push_back(sensor_value);


                // we gotta check if the sensor is visible
                if(pose_changed[i] && nh->hasParam("check_visibility") && has_mesh){
                    Vector3d ray(sensor_pos[0],sensor_pos[1],sensor_pos[2]);
                    sensor_visible[i][j] = checkIfSensorVisible(meshes[i].vertices_transformed,meshes[i].polygons,ray);
                }

                j++;
            }

            if (!msg.sensor_value.empty())
                sensors_pub.publish(msg);
        }
        rate.sleep();
        motor = !motor;
    }
}

void LighthouseSimulator::PublishImuData() {
    ros::Duration d(3);
    d.sleep();
    ros::Rate rate(1);
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while (imu_publishing) {
        for(int i=0;i<meshes.size();i++){
            Matrix4d RT_object2world;
            if (!getTransform(name[i].c_str(), "world", RT_object2world))
                continue;

            sensor_msgs::Imu msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = name[i];
            msg.orientation.w = 1;
            msg.linear_acceleration_covariance = {
                    1, 0, 0,
                    0, 1, 0,
                    0, 0, 1
            };

            msg.linear_acceleration.x = 0 + rand()/(double)RAND_MAX * IMU_ACC_NOISE;
            msg.linear_acceleration.y = 0 + rand()/(double)RAND_MAX * IMU_ACC_NOISE;
            msg.linear_acceleration.z = 9.81 + rand()/(double)RAND_MAX * IMU_ACC_NOISE;

            imu_pub[i].publish(msg);
            rate.sleep();
        }
    }
}

bool LighthouseSimulator::checkIfSensorVisible(vector<Vector4d> &vertices, vector<::pcl::Vertices> &polygons, Vector3d &ray){
    // the ray origin is the lighthouse origin
    Vector3d origin(0,0,0);
    int triangle_intersections = 0;
    int messageID = 6000+1000*id;
    for(auto polygon:polygons){
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
