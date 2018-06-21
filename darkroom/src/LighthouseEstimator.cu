#include "darkroom/LighthouseEstimator.cuh"

LighthouseEstimatorCUDA::LighthouseEstimatorCUDA() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LighthouseEstimatorCUDA",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    sensor_location_pub = nh->advertise<roboy_communication_middleware::DarkRoomSensor>(
            "/roboy/middleware/DarkRoom/sensor_location", 1);
    lighthouse_pose_correction = nh->advertise<roboy_communication_middleware::LighthousePoseCorrection>(
            "/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    tracking = false;
    calibrating = false;
    poseestimating = false;
    distances = false;
    rays = false;
    particle_filtering = false;
    use_lighthouse_calibration_data_phase[LIGHTHOUSE_A] = false;
    use_lighthouse_calibration_data_phase[LIGHTHOUSE_B] = false;
    use_lighthouse_calibration_data_tilt[LIGHTHOUSE_A] = false;
    use_lighthouse_calibration_data_tilt[LIGHTHOUSE_B] = false;
    use_lighthouse_calibration_data_gibphase[LIGHTHOUSE_A] = false;
    use_lighthouse_calibration_data_gibphase[LIGHTHOUSE_B] = false;
    use_lighthouse_calibration_data_gibmag[LIGHTHOUSE_A] = false;
    use_lighthouse_calibration_data_gibmag[LIGHTHOUSE_B] = false;

//    object_pose = VectorXd(6);
//    object_pose << 0, 0, 0, 0, 0, 0.001;
}

void LighthouseEstimatorCUDA::getVisibleCalibratedSensors(vector<int> &visible_sensors) {
    for (auto &sensor : sensors) {
        if (sensor.second.isActive(LIGHTHOUSE_A) &&
            sensor.second.isActive(LIGHTHOUSE_B) &&
            sensor.second.isCalibrated())
            visible_sensors.push_back(sensor.first);
    }
}

void LighthouseEstimatorCUDA::getVisibleCalibratedSensors(bool lighthouse, vector<int> &visible_sensors) {
    for (auto &sensor : sensors) {
        if (sensor.second.isActive(lighthouse) && sensor.second.isCalibrated())
            visible_sensors.push_back(sensor.first);
    }
}

bool LighthouseEstimatorCUDA::estimateSensorPositionsUsingRelativeDistances(bool lighthouse, vector<int> &specificIds) {
    ROS_INFO_STREAM("estimating distance of sensors to lighthouse " << lighthouse + 1);
    vector<Vector3d> relPos;
    vector<double> elevations, azimuths;
    vector<double> distanceToLighthouse;
    vector<int> ids;
    if (specificIds.empty()) {
        // let's see who is active
        cout << "using sensors:" << endl;
        for (auto &sensor : sensors) {
            // skip inactive/uncalibrated sensors
            if (sensor.second.isActive(lighthouse) && sensor.second.isCalibrated()) {
                ids.push_back(sensor.first);
                sensor.second.get(lighthouse, elevations, azimuths);
                // apply factory calibration correction if desired
                applyCalibrationData(lighthouse, elevations.back(), azimuths.back());
                sensor.second.getRelativeLocation(relPos);
                distanceToLighthouse.push_back(sensor.second.getDistance(lighthouse));
                cout << sensor.first << "\t";
            }
        }
        cout << endl;
    } else {
        uint sensor_counter = 0;
        for (uint i = 0; i < specificIds.size(); i++) {
            // skip inactive sensors
            if (sensors[specificIds.at(i)].isActive(lighthouse)) {
                sensor_counter++;
            } else {
                ROS_WARN_THROTTLE(1, "sensor%d inactive", specificIds.at(i));
            }
        }
        if (sensor_counter < specificIds.size()) {
            ROS_WARN("time out waiting for specific sensors");
            return false;
        }
        // get the values now that all requested sensors are active
        for (uint i = 0; i < specificIds.size(); i++) {
            ids.push_back(specificIds.at(i));
            sensors[specificIds.at(i)].get(lighthouse, elevations, azimuths);
            // apply factory calibration correction if desired
            // apply factory calibration correction if desired
            applyCalibrationData(lighthouse, elevations.back(), azimuths.back());
            sensors[specificIds.at(i)].getRelativeLocation(relPos);
            distanceToLighthouse.push_back(sensors[specificIds.at(i)].getDistance(lighthouse));
        }
    }

    if (ids.size() < 3)
        return false;
    // cost function
    auto f = [](double &R0, double &R1, double &cosine, double &distance) {
        return (pow(R0, 2.0) + pow(R1, 2.0) - 2.0 * R0 * R1 * cosine - pow(distance, 2.0));
    };
    // partial derivative
    auto df = [](double &R0, double &R1, double &cosine) { return (2.0 * R0 - 2 * R1 * cosine); };

    MatrixXd cosineBetween(ids.size(), ids.size()), distanceBetween(ids.size(), ids.size());
    for (uint i = 0; i < ids.size() - 1; i++) {
        for (uint j = i + 1; j < ids.size(); j++) {
            // calculate the cosine between the two sensors
            cosineBetween(i, j) =
                    sin(elevations[i]) * cos(azimuths[i]) * sin(elevations[j]) * cos(azimuths[j]) +
                    sin(elevations[i]) * sin(azimuths[i]) * sin(elevations[j]) * sin(azimuths[j]) +
                    cos(elevations[i]) * cos(elevations[j]);

            ROS_DEBUG("cosine between %d and %d: %f", i, j, cosineBetween(i, j));
            // calculate the distance between the sensors
            distanceBetween(i, j) = (relPos[i] - relPos[j]).norm();
            ROS_DEBUG("distance between sensor %d and %d: %f", ids[i], ids[j], distanceBetween(i, j));
        }
    }

    int iterations = 0;

    uint n = ids.size();
    MatrixXd J(n * (n - 1) / 2, n);
    J = J.setZero();
    VectorXd v(n * (n - 1) / 2), d_old(ids.size());

    double error, error_prev = 10000000;
    while (iterations < MAX_ITERATIONS) {

        // construct jacobian and function vector
        int row = 0;
        for (uint i = 0; i < ids.size() - 1; i++) {
            for (uint j = i + 1; j < ids.size(); j++) {
                J(row, i) = df(distanceToLighthouse[i], distanceToLighthouse[j], cosineBetween(i, j));
                J(row, j) = df(distanceToLighthouse[j], distanceToLighthouse[i], cosineBetween(i, j));
                v(row) = f(distanceToLighthouse[i], distanceToLighthouse[j], cosineBetween(i, j),
                           distanceBetween(i, j));
                row++;
            }
        }
        for (uint i = 0; i < ids.size(); i++) {
            d_old(i) = distanceToLighthouse[i];
        }
//        if (iterations % 100 == 0) {
//            ROS_INFO_STREAM("J\n" << J);
//            ROS_INFO_STREAM("v\n" << v);
//            ROS_INFO_STREAM("d_old\n" << d_old);
//        }

        error = v.norm() / (double) ids.size();
        if (error < ERROR_THRESHOLD || (error_prev - error) < 0.00000001) {
            break;
        }
        error_prev = error;
        ROS_INFO_THROTTLE(5, "iteration %d error %lf", iterations, error);
        // construct distance new vector, sharing data with the stl container
        Map<VectorXd> d_new(distanceToLighthouse.data(), distanceToLighthouse.size());
//        d_new = d_old - (J.transpose() * J).inverse() * J.transpose() * v;
        iterations++;
    }

    uint i = 0;
    for (auto id:ids) {
        ROS_DEBUG_STREAM("sensor:" << id << " distance to lighthouse " << lighthouse << ": " << d_old(i));

        Vector2d angles(elevations[i], azimuths[i]);
        Eigen::Vector3d u0;
        rayFromLighthouseAngles(angles, u0, lighthouse);

        Vector3d relLocation(d_old(i) * u0(0), d_old(i) * u0(1), d_old(i) * u0(2));
        sensors[id].set(lighthouse, relLocation);

        i++;

        char str[100];
        sprintf(str, "sensor_%d_estimated", id);

        publishSphere(relLocation, (lighthouse ? "lighthouse2" : "lighthouse1"), str,
                      getMessageID(DISTANCE, id, lighthouse), COLOR(0, 1, lighthouse ? 0 : 1, 0.3), 0.01f, 0);

        sprintf(str, "ray_%d", id);
        Vector3d pos(0, 0, 0);
        publishRay(pos, relLocation, (lighthouse ? "lighthouse2" : "lighthouse1"), str,
                   getMessageID(RAY, id, lighthouse), COLOR(0, 1, lighthouse ? 0 : 1, 0.3), 0);

        sprintf(str, "%d", id);
        publishText(relLocation, str, (lighthouse ? "lighthouse2" : "lighthouse1"), "sensor_id", rand(),
                    COLOR(1, 0, 0, 0.5), 0, 0.04f);
    }

    for (auto id:ids) {
        for (auto id2:ids) {
            if (id2 != id) {
                Vector3d pos1, pos2, dir;
                sensors[id].get(lighthouse, pos1);
                sensors[id2].get(lighthouse, pos2);
                dir = pos2 - pos1;
                publishRay(pos1, dir, (lighthouse ? "lighthouse2" : "lighthouse1"), "distance",
                           rand(), COLOR(0, 1, lighthouse ? 0 : 1, 0.5), 0);

                if (distances) {
                    char str[100];
                    sprintf(str, "%.3f", dir.norm());
                    Vector3d pos = pos1 + dir / 2.0;
                    publishText(pos, str, (lighthouse ? "lighthouse2" : "lighthouse1"), "distance", rand(),
                                COLOR(1, 0, 0, 0.5), 0, 0.02f);
                }
            }
        }
    }

    if (iterations < MAX_ITERATIONS)
        if (error < ERROR_THRESHOLD) {
            ROS_WARN_STREAM(
                    "mean squared error " << error << " below threshold " << ERROR_THRESHOLD << " in " << iterations
                                          << " iterations for lighthouse " << lighthouse + 1);
        } else if ((error - error_prev) < 0.00000001) {
            ROS_WARN_STREAM(
                    "mean squared error " << error << " previous " << error_prev << " doesn't get lower after "
                                          << iterations << " iterations for lighthouse " << lighthouse + 1
            );
        } else
            ROS_WARN_STREAM(
                    "maximal number of iterations reached, mean squared error " << error << " in " << iterations
                                                                                << " iterations for lighthouse "
                                                                                << lighthouse + 1
            );
    return true;
}

void LighthouseEstimatorCUDA::triangulateSensors() {
//    high_resolution_clock::time_point timestamp_new[4];
//    map<int, high_resolution_clock::time_point[4]> timestamps_old;
//
//    ros::Rate rate(30);
//    bool lighthouse_active[2];
//    while (tracking) {
//        roboy_communication_middleware::DarkRoomSensor msg;
//
//        Matrix4d RT_0, RT_1;
//        if (!getTransform(LIGHTHOUSE_A, "world", RT_0)) {
//            rate.sleep(); // no need to query for frame faster than it is published
//            continue;
//        }
//        if (!getTransform(LIGHTHOUSE_B, "world", RT_1)) {
//            rate.sleep(); // no need to query for frame faster than it is published
//            continue;
//        }
//
//        int active_sensors_counter = 0;
//
//        for (auto &sensor : sensors) {
//            lighthouse_active[LIGHTHOUSE_A] = sensor.second.isActive(LIGHTHOUSE_A);
//            lighthouse_active[LIGHTHOUSE_B] = sensor.second.isActive(LIGHTHOUSE_B);
//            if (sensor.second.hasNewData(timestamps_old[sensor.first])) {
//                Vector2d lighthouse0_angles;
//                Vector2d lighthouse1_angles;
//                sensor.second.get(LIGHTHOUSE_A, lighthouse0_angles, &timestamp_new[LIGHTHOUSE_A * 2]);
//                sensor.second.get(LIGHTHOUSE_B, lighthouse1_angles, &timestamp_new[LIGHTHOUSE_B * 2]);
//
//                memcpy(timestamps_old[sensor.first], timestamp_new, sizeof(timestamp_new));
//
//                Vector3d ray0, ray1;
//
//                if (lighthouse_active[LIGHTHOUSE_A] && lighthouse_active[LIGHTHOUSE_B]) {
//                    active_sensors_counter++;
//
//                    Vector3d triangulated_position;
//
//                    applyCalibrationData(lighthouse0_angles, lighthouse1_angles);
//
//                    triangulateFromLighthouseAngles(lighthouse0_angles, lighthouse1_angles, RT_0, RT_1,
//                                                    triangulated_position, ray0,
//                                                    ray1);
//
//                    sensor.second.set(triangulated_position);
//
//                    if (!triangulated_position.hasNaN()) {
//                        char str[100], str2[2];
//                        sprintf(str, "sensor_%d", sensor.first);
//                        publishSphere(triangulated_position, "world", str,
//                                      getMessageID(TRIANGULATED, sensor.first), COLOR(0, 1, 0, 0.8), 0.01f, 1);
//                        sprintf(str2, "%d", sensor.first);
//                        publishText(triangulated_position, str2, "world", str, getMessageID(SENSOR_NAME, sensor.first),
//                                    COLOR(1, 1, 1, 0.7), 0.1, 0.04f);
//                        msg.ids.push_back(sensor.first);
//                        geometry_msgs::Vector3 v;
//                        v.x = triangulated_position[0];
//                        v.y = triangulated_position[1];
//                        v.z = triangulated_position[2];
//                        msg.position.push_back(v);
//                    }
//
//                    if (rays) {
//                        Vector3d pos(0, 0, 0);
//                        ray0 *= 5;
//                        publishRay(pos, ray0, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
//                                   COLOR(0, 1, 0, 1.0), 1);
//                        ray1 *= 5;
//                        publishRay(pos, ray1, "lighthouse2", "rays_lighthouse_2", getMessageID(RAY, sensor.first, 1),
//                                   COLOR(0, 1, 0, 1.0), 1);
//
//                    }
//                }
//
//                if (distances) {
//                    int id = 0;
//                    for (auto &sensor_other : sensors) {
//                        if (sensor.first != sensor_other.first &&
//                            (sensor_other.second.isActive(0) && sensor_other.second.isActive(1))) {
//                            Vector3d pos1, pos2, dir;
//                            sensor_other.second.getPosition3D(pos2);
//                            sensor.second.getPosition3D(pos1);
//                            dir = pos2 - pos1;
//                            publishRay(pos1, dir, "world", "distance",
//                                       getMessageID(DISTANCES, id++), COLOR(0, 1, 1, 1.0), 1);
//
//                            char str[100];
//                            sprintf(str, "%.3f", dir.norm());
//                            Vector3d pos = pos1 + dir / 2.0;
//                            publishText(pos, str, "world", "distance", getMessageID(DISTANCES, id++),
//                                        COLOR(1, 0, 0, 0.5), 1, 0.02f);
//                        }
//                    }
//                }
//            }
//        }
//        active_sensors = active_sensors_counter;
//        if (msg.ids.size() > 0)
//            sensor_location_pub.publish(msg);
//    }
}

void LighthouseEstimatorCUDA::publishRays() {
//    high_resolution_clock::time_point timestamp_new[4];
//    map<int, high_resolution_clock::time_point[4]> timestamps_old;
//
//    ros::Rate rate(30);
//    bool lighthouse_active[2];
//
//    while (rays) {
//        for (auto &sensor : sensors) {
//            lighthouse_active[LIGHTHOUSE_A] = sensor.second.isActive(LIGHTHOUSE_A);
//            lighthouse_active[LIGHTHOUSE_B] = sensor.second.isActive(LIGHTHOUSE_B);
//            if (sensor.second.hasNewData(timestamps_old[sensor.first])) {
//                Vector2d lighthouse0_angles;
//                Vector2d lighthouse1_angles;
//                sensor.second.get(LIGHTHOUSE_A, lighthouse0_angles, &timestamp_new[LIGHTHOUSE_A * 2]);
//                sensor.second.get(LIGHTHOUSE_B, lighthouse1_angles, &timestamp_new[LIGHTHOUSE_B * 2]);
//
//                memcpy(timestamps_old[sensor.first], timestamp_new, sizeof(timestamp_new));
//
//                if (lighthouse_active[LIGHTHOUSE_A]) {
//                    Vector3d ray;
//                    rayFromLighthouseAngles(lighthouse0_angles, ray, LIGHTHOUSE_A);
//                    Vector3d pos(0, 0, 0);
//                    ray *= 5;
//                    publishRay(pos, ray, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
//                               COLOR(1, 0, 0, 0.5), 1);
//                }
//                if (lighthouse_active[LIGHTHOUSE_B]) {
//                    Vector3d ray;
//                    rayFromLighthouseAngles(lighthouse1_angles, ray, LIGHTHOUSE_B);
//                    Vector3d pos(0, 0, 0);
//                    ray *= 5;
//                    publishRay(pos, ray, "lighthouse2", "rays_lighthouse_2", getMessageID(RAY, sensor.first, 0),
//                               COLOR(1, 0, 0, 0.5), 1);
//                }
//            }
//        }
//        rate.sleep();
//    }
}

int LighthouseEstimatorCUDA::getMessageID(int type, int sensor, bool lighthouse) {
//    TRIANGULATED = 0,      // for each sensor
//    DISTANCE = 1,           // for each sensor and lighthouse
//    RAY = 2,   // for each sensor and lighthouse
//    SENSOR_NAME = 3,   // for each sensor
//    DISTANCES = 4
    int n_sensors = sensors.size(), per_lighthouse = n_sensors * NUMBER_OF_LIGHTHOUSES * lighthouse;


    switch (type) {
        case TRIANGULATED:
            return sensor;
        case DISTANCE:
            return n_sensors + per_lighthouse + sensor;
        case RAY:
            return n_sensors + n_sensors * NUMBER_OF_LIGHTHOUSES + n_sensors + per_lighthouse + sensor;
        case SENSOR_NAME:
            return n_sensors + 2 * n_sensors * NUMBER_OF_LIGHTHOUSES + sensor;
        case DISTANCES:
            return 6000 + sensor;
        default:
            return rand();
    }
}

void LighthouseEstimatorCUDA::applyCalibrationData(Vector2d &lighthouse0_angles, Vector2d &lighthouse1_angles) {
    applyCalibrationData(LIGHTHOUSE_A, lighthouse0_angles);
    applyCalibrationData(LIGHTHOUSE_B, lighthouse1_angles);
}

void LighthouseEstimatorCUDA::applyCalibrationData(bool lighthouse, Vector2d &lighthouse_angles) {
    if (use_lighthouse_calibration_data_phase[lighthouse]) {
        lighthouse_angles[HORIZONTAL] += ootx[lighthouse].fcal_0_phase;
        lighthouse_angles[VERTICAL] += ootx[lighthouse].fcal_1_phase;
    }
    if (use_lighthouse_calibration_data_tilt[lighthouse]) {
        lighthouse_angles[HORIZONTAL] += ootx[lighthouse].fcal_0_tilt;
        lighthouse_angles[VERTICAL] += ootx[lighthouse].fcal_1_tilt;
    }
    if (use_lighthouse_calibration_data_gibphase[lighthouse]) {
        // TODO
    }
    if (use_lighthouse_calibration_data_gibmag[lighthouse]) {
        // TODO
    }
}

void LighthouseEstimatorCUDA::applyCalibrationData(bool lighthouse, double &elevation, double &azimuth) {
    if (use_lighthouse_calibration_data_phase[lighthouse]) {
        azimuth += ootx[lighthouse].fcal_0_phase;
        elevation += ootx[lighthouse].fcal_1_phase;
    }
    if (use_lighthouse_calibration_data_tilt[lighthouse]) {
        azimuth += ootx[lighthouse].fcal_0_tilt;
        elevation += ootx[lighthouse].fcal_1_tilt;
    }
    if (use_lighthouse_calibration_data_gibphase[lighthouse]) {
        // TODO
    }
    if (use_lighthouse_calibration_data_gibmag[lighthouse]) {
        // TODO
    }
}
