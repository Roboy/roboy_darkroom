#include "darkroom/LighthouseEstimator.hpp"

LighthouseEstimator::LighthouseEstimator() {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "LighthouseEstimator",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }
    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    sensor_location_pub = nh->advertise<roboy_communication_middleware::DarkRoomSensor>(
            "/roboy/middleware/DarkRoom/sensor_location", 1);
    lighthouse_pose_correction = nh->advertise<roboy_communication_middleware::LighthousePoseCorrection>(
            "/roboy/middleware/DarkRoom/LighthousePoseCorrection", 1);
    pose_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(
            "/roboy/middleware/pose0", 1);
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();

    tracking = false;
    calibrating = false;
    poseestimating = false;
    distances = false;
    rays = false;
    particle_filtering = false;

    object_pose = VectorXd(6);
    object_pose << 0, 0, 0, 0, 0, 0.001;
}

void LighthouseEstimator::getVisibleCalibratedSensors(vector<int> &visible_sensors) {
    for (auto &sensor : sensors) {
        if (sensor.second.isActive(LIGHTHOUSE_A) &&
            sensor.second.isActive(LIGHTHOUSE_B) &&
            sensor.second.isCalibrated())
            visible_sensors.push_back(sensor.first);
    }
}

void LighthouseEstimator::getVisibleCalibratedSensors(bool lighthouse, vector<int> &visible_sensors) {
    for (auto &sensor : sensors) {
        if (sensor.second.isActive(lighthouse) && sensor.second.isCalibrated())
            visible_sensors.push_back(sensor.first);
    }
}

void LighthouseEstimator::calibrateRelativeSensorDistances() {
    map<int, vector<Vector3d>> sensorPosition3d;
    map<int, int> number_of_samples;

    // measure the sensor location for a couple of seconds
    ros::Time start_time = ros::Time::now();
    clearAll();

    high_resolution_clock::time_point timestamp0_new[2], timestamp1_new[2];
    map<int, high_resolution_clock::time_point[2]> timestamps0_old, timestamps1_old;

    clearAll();
    ROS_INFO("measuring mean sensor positions for 10 seconds");
    int message_counter = 0;

    // get the lighthouse poses
    Matrix4d RT_0, RT_1;
    if (!getTransform("world", LIGHTHOUSE_A, RT_0))
        return;
    if (!getTransform("world", LIGHTHOUSE_B, RT_1))
        return;

    while ((ros::Time::now() - start_time) < ros::Duration(10) && calibrating) {
        for (auto &sensor : sensors) {
            // if the sensor is visible for both lighthouses and it is active
            if (sensor.second.isActive(0) && sensor.second.isActive(1)) {
                Vector2d lighthouse0_angles;
                Vector2d lighthouse1_angles;
                // triangulate the locations
                sensor.second.get(0, lighthouse0_angles, timestamp0_new);
                sensor.second.get(1, lighthouse1_angles, timestamp1_new);

                // check if this is actually new data
                if (timestamps0_old[sensor.first][0] != timestamp0_new[0] &&
                    timestamps0_old[sensor.first][1] != timestamp0_new[1] &&
                    timestamps1_old[sensor.first][0] != timestamp1_new[0] &&
                    timestamps1_old[sensor.first][1] != timestamp1_new[1]) {

                    timestamps0_old[sensor.first][0] = timestamp0_new[0];
                    timestamps0_old[sensor.first][1] = timestamp0_new[1];
                    timestamps1_old[sensor.first][0] = timestamp1_new[0];
                    timestamps1_old[sensor.first][1] = timestamp1_new[1];

                    Vector3d position, ray0, ray1;
                    triangulateFromLighthouseAngles(lighthouse0_angles, lighthouse1_angles, RT_0, RT_1, position, ray0,
                                                    ray1);

                    if (sensorPosition3d[sensor.first].size() > 0) {
                        Vector3d diff = sensorPosition3d[sensor.first].back() - position;
                        if (diff.norm() < 0.1) { // reject outliers
                            sensorPosition3d[sensor.first].push_back(position);
                            number_of_samples[sensor.first]++;
                        }
                    } else {
                        sensorPosition3d[sensor.first].push_back(position);
                        number_of_samples[sensor.first]++;
                    }

                    publishSphere(position, "world", "sensor_calibration", message_counter++, COLOR(1, 0, 0, 0.1),
                                  0.005f, 100);

                }
            }
        }
    }

    // calculate the mean and covariance for each sensor and origin of the object (the origin is the average center)
    Vector3d origin(0, 0, 0);
    map<int, Vector3d> mean;
    map<int, Vector3d> variance;
    map<int, bool> sensor_accepted;
    int active_sensors = 0;

    // set the relative sensor location for each sensor wrt the origin
    for (auto const &sensor : sensors) {
        if (number_of_samples[sensor.first] > 200) {
            mean[sensor.first] = Vector3d(0.0, 0.0, 0.0);
            for (auto pos:sensorPosition3d[sensor.first]) {
                mean[sensor.first] += pos;
            }
            mean[sensor.first] /= number_of_samples[sensor.first];
            variance[sensor.first] = Vector3d(0.0, 0.0, 0.0);
            for (auto pos:sensorPosition3d[sensor.first]) {
                variance[sensor.first](0) += pow(pos.x() - mean[sensor.first].x(), 2.0);
                variance[sensor.first](1) += pow(pos.y() - mean[sensor.first].y(), 2.0);
                variance[sensor.first](2) += pow(pos.z() - mean[sensor.first].z(), 2.0);
            }
            variance[sensor.first] /= number_of_samples[sensor.first];
            ROS_INFO_STREAM("sensor " << sensor.first << " mean("
                                      << mean[sensor.first][0] << ", " << mean[sensor.first][1] << ", "
                                      << mean[sensor.first][2] << ") variance("
                                      << variance[sensor.first][0] << ", " << variance[sensor.first][1] << ", "
                                      << variance[sensor.first][2] << ")"
                                      << " " << "number of samples" << number_of_samples[sensor.first]);
            if (variance[sensor.first].norm() > 0) {
                sensor_accepted[sensor.first] = true;
                active_sensors++;
                origin += mean[sensor.first];
            } else {
                sensor_accepted[sensor.first] = false;
            }
        } else {
            ROS_INFO("rejecting sensor %d, because it does not have enough samples (%d)", sensor.first,
                     number_of_samples[sensor.first]);
        }
    }
    if (active_sensors == 0) {
        ROS_WARN("no active sensors, aborting");
        return;
    }
    origin /= (double) active_sensors;
    if (origin.hasNaN()) {
        ROS_WARN("origin not finite, aborting");
        return;
    }
    publishSphere(origin, "world", "origin", message_counter++, COLOR(0, 0, 1, 1.0), 0.01f);
    for (auto &sensor : sensors) {
        if (sensor_accepted[sensor.first]) {
            Vector3d rel;
            rel = mean[sensor.first] - origin;
            sensor.second.setRelativeLocation(rel);
            ROS_INFO_STREAM("origin(" << origin[0] << ", " << origin[1] << ", " << origin[2] << ")");
            publishSphere(mean[sensor.first], "world", "mean", message_counter++, COLOR(0, 0, 1, 1.0), 0.01f);
            publishRay(origin, rel, "world", "relative", message_counter++, COLOR(1, 1, 0, 1.0));
        }
    }
}

bool LighthouseEstimator::estimateSensorPositionsUsingRelativeDistances(bool lighthouse, vector<int> &specificIds) {
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

    double error;
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
        if (error < ERROR_THRESHOLD) {
            break;
        }
        // construct distance new vector, sharing data with the stl container
        Map<VectorXd> d_new(distanceToLighthouse.data(), distanceToLighthouse.size());
        d_new = d_old - (J.transpose() * J).inverse() * J.transpose() * v;
        iterations++;
    }

    uint i = 0;
    for (auto id:ids) {
        ROS_DEBUG_STREAM("sensor:" << id << " distance to lighthouse " << lighthouse << ": " << d_old(i));

        Vector2d angles(elevations[i], azimuths[i]);
        Eigen::Vector3d u0;
        rayFromLighthouseAngles(angles, u0);

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
        ROS_WARN_STREAM(
                "mean squared error " << error << " below threshold " << ERROR_THRESHOLD << " in " << iterations
                                      << " iterations");
    else
        ROS_WARN_STREAM(
                "maximal number of iterations reached, mean squared error " << error << " in " << iterations
                                                                            << " iterations");
    return true;
}

bool LighthouseEstimator::estimateObjectPoseUsingRelativeDistances(){
    vector<int> visible_sensors[2];
    getVisibleCalibratedSensors(LIGHTHOUSE_A, visible_sensors[LIGHTHOUSE_A]);
    VectorXd pose[2];
    Matrix4d  RT_object[2];
    if (visible_sensors[LIGHTHOUSE_A].size() >= 4){
        if(estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A,visible_sensors[LIGHTHOUSE_A])) {
            PoseEstimatorSensorCloud::PoseEstimator estimator(visible_sensors[LIGHTHOUSE_A].size());
            uint i = 0;
            for (auto sensor:visible_sensors[LIGHTHOUSE_A]) {
                Vector3d position3d, relLocation1;
                sensors[sensor].get(LIGHTHOUSE_A, position3d); // get the relative position for this lighthouse
                sensors[sensor].getRelativeLocation(relLocation1);
                estimator.pos3D_A.block(0, i, 4, 1) << position3d(0), position3d(1), position3d(2), 1;
                estimator.pos3D_B.block(0, i, 4, 1) << relLocation1(0), relLocation1(1), relLocation1(2), 1;
                i++;
            }

            pose[LIGHTHOUSE_A] = VectorXd(6);
            pose[LIGHTHOUSE_A] << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;

            NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator> *numDiff;
            Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double> *lm;
            numDiff = new NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>(estimator);
            lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double>(*numDiff);
            lm->parameters.maxfev = MAX_ITERATIONS;
            lm->parameters.xtol = 1e-10;
            int ret = lm->minimize(pose[LIGHTHOUSE_A]);
            ROS_INFO_THROTTLE(1,
                              "object pose estimation using %ld sensors, finished after %ld iterations, with an error of %f",
                              visible_sensors[LIGHTHOUSE_A].size(), lm->iter, lm->fnorm);

            getRTmatrix(RT_object[LIGHTHOUSE_A], pose[LIGHTHOUSE_A]);

            tf::Transform tf;
            getTFtransform(RT_object[LIGHTHOUSE_A], tf);
            publishTF(tf, (LIGHTHOUSE_A?"lighthouse2":"lighthouse1"), "object_lighthouse_1");
        }
    }

    getVisibleCalibratedSensors(LIGHTHOUSE_B, visible_sensors[LIGHTHOUSE_B]);
    if (visible_sensors[LIGHTHOUSE_B].size() >= 4){
        if(estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B,visible_sensors[LIGHTHOUSE_B])) {
            PoseEstimatorSensorCloud::PoseEstimator estimator(visible_sensors[LIGHTHOUSE_B].size());
            uint i = 0;
            for (auto sensor:visible_sensors[LIGHTHOUSE_B]) {
                Vector3d position3d, relLocation1;
                sensors[sensor].get(LIGHTHOUSE_B, position3d); // get the relative position for this lighthouse
                sensors[sensor].getRelativeLocation(relLocation1);
                estimator.pos3D_A.block(0, i, 4, 1) << position3d(0), position3d(1), position3d(2), 1;
                estimator.pos3D_B.block(0, i, 4, 1) << relLocation1(0), relLocation1(1), relLocation1(2), 1;
                i++;
            }

            pose[LIGHTHOUSE_B] = VectorXd(6);
            pose[LIGHTHOUSE_B] << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
            pose[LIGHTHOUSE_B] << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;

            NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator> *numDiff;
            Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double> *lm;
            numDiff = new NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>(estimator);
            lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double>(*numDiff);
            lm->parameters.maxfev = MAX_ITERATIONS;
            lm->parameters.xtol = 1e-10;
            int ret = lm->minimize(pose[LIGHTHOUSE_B]);
            ROS_INFO_THROTTLE(1,
                              "object pose estimation using %ld sensors, finished after %ld iterations, with an error of %f",
                              visible_sensors[LIGHTHOUSE_B].size(), lm->iter, lm->fnorm);

            getRTmatrix(RT_object[LIGHTHOUSE_B], pose[LIGHTHOUSE_B]);

            tf::Transform tf;
            getTFtransform(RT_object[LIGHTHOUSE_B], tf);
            publishTF(tf, (LIGHTHOUSE_B?"lighthouse2":"lighthouse1"), "object_lighthouse_2");
        }
    }

    Matrix4d RT_correct = RT_object[LIGHTHOUSE_A]*RT_object[LIGHTHOUSE_B].inverse();

    tf::Transform tf;
    tf.setOrigin(tf::Vector3(RT_correct(0,3), RT_correct(1,3), RT_correct(2,3)));
    tf::Matrix3x3 rot_matrix(RT_correct(0, 0), RT_correct(0, 1), RT_correct(0, 2),
                             RT_correct(1, 0), RT_correct(1, 1), RT_correct(1, 2),
                             RT_correct(2, 0), RT_correct(2, 1), RT_correct(2, 2));

    tf.setBasis(rot_matrix);

    roboy_communication_middleware::LighthousePoseCorrection msg;
    msg.id = LIGHTHOUSE_B;
    msg.type = ABSOLUT;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);

//
//
//    Quaterniond q;
//    Vector3d origin;
//    getPose(q,origin,object_pose);
//
//    publishMesh("darkroom","calibrated_objects/models", mesh.c_str(), origin, q, 0.001, "world", "mesh", 9999, 1);

//        if(lm->fnorm>0.1) // arbitrary but very bad
//            object_pose << 0, 0, 0, 0, 0, 0.1;

}

void LighthouseEstimator::triangulateSensors() {
    high_resolution_clock::time_point timestamp_new[4];
    map<int, high_resolution_clock::time_point[4]> timestamps_old;

    ros::Rate rate(30);
    bool lighthouse_active[2];
    while (tracking) {
        roboy_communication_middleware::DarkRoomSensor msg;

        Matrix4d RT_0, RT_1;
        if (!getTransform(LIGHTHOUSE_A, "world", RT_0)) {
            rate.sleep(); // no need to query for frame faster than it is published
            continue;
        }
        if (!getTransform(LIGHTHOUSE_B, "world", RT_1)) {
            rate.sleep(); // no need to query for frame faster than it is published
            continue;
        }

        for (auto &sensor : sensors) {
            lighthouse_active[LIGHTHOUSE_A] = sensor.second.isActive(LIGHTHOUSE_A);
            lighthouse_active[LIGHTHOUSE_B] = sensor.second.isActive(LIGHTHOUSE_B);
            if (sensor.second.hasNewData(timestamps_old[sensor.first])) {
                Vector2d lighthouse0_angles;
                Vector2d lighthouse1_angles;
                sensor.second.get(LIGHTHOUSE_A, lighthouse0_angles, &timestamp_new[LIGHTHOUSE_A*2]);
                sensor.second.get(LIGHTHOUSE_B, lighthouse1_angles, &timestamp_new[LIGHTHOUSE_B*2]);

                memcpy(timestamps_old[sensor.first],timestamp_new,sizeof(timestamp_new));

                Vector3d ray0, ray1;

                if(lighthouse_active[LIGHTHOUSE_A] && lighthouse_active[LIGHTHOUSE_B] ) {
                    Vector3d triangulated_position;
                    triangulateFromLighthouseAngles(lighthouse0_angles, lighthouse1_angles, RT_0, RT_1,
                                                    triangulated_position, ray0,
                                                    ray1);

                    sensor.second.set(triangulated_position);

                    if (!triangulated_position.hasNaN()) {
                        char str[100], str2[2];
                        sprintf(str, "sensor_%d", sensor.first);
                        publishSphere(triangulated_position, "world", str,
                                      getMessageID(TRIANGULATED, sensor.first), COLOR(0, 1, 0, 0.8), 0.01f, 0.1);
                        sprintf(str2, "%d", sensor.first);
                        publishText(triangulated_position, str2, "world", str, getMessageID(SENSOR_NAME, sensor.first),
                                    COLOR(1, 1, 1, 0.7), 0.1, 0.04f);
                        msg.ids.push_back(sensor.first);
                        geometry_msgs::Vector3 v;
                        v.x = triangulated_position[0];
                        v.y = triangulated_position[1];
                        v.z = triangulated_position[2];
                        msg.position.push_back(v);
                    }

                    if (rays) {
                        Vector3d pos(0, 0, 0);
                        ray0 *= 5;
                        publishRay(pos, ray0, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
                                   COLOR(0, 1, 0, 1.0), 0.1);
                        ray1 *= 5;
                        publishRay(pos, ray1, "lighthouse2", "rays_lighthouse_2", getMessageID(RAY, sensor.first, 1),
                                   COLOR(0, 1, 0, 1.0), 0.1);

                    }
                }else{
                    if(lighthouse_active[LIGHTHOUSE_A] && rays) {
                        rayFromLighthouseAngles(lighthouse0_angles, ray0);
                        Vector3d pos(0, 0, 0);
                        ray0 *= 5;
                        publishRay(pos, ray0, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
                                   COLOR(1, 0, 0, 0.3), 0.1);
                    }
                    if(lighthouse_active[LIGHTHOUSE_B] && rays) {
                        rayFromLighthouseAngles(lighthouse1_angles, ray1);
                        Vector3d pos(0, 0, 0);
                        ray1 *= 5;
                        publishRay(pos, ray1, "lighthouse2", "rays_lighthouse_2", getMessageID(RAY, sensor.first, 0),
                                   COLOR(1, 0, 0, 0.3), 0.1);
                    }
                }

                if (distances) {
                    int id = 0;
                    for (auto &sensor_other : sensors) {
                        if (sensor.first != sensor_other.first &&
                            (sensor_other.second.isActive(0) && sensor_other.second.isActive(1))) {
                            Vector3d pos1, pos2, dir;
                            sensor_other.second.getPosition3D(pos2);
                            sensor.second.getPosition3D(pos1);
                            dir = pos2 - pos1;
                            publishRay(pos1, dir, "world", "distance",
                                       getMessageID(DISTANCES, id++), COLOR(0, 1, 1, 1.0), 1);

                            char str[100];
                            sprintf(str, "%.3f", dir.norm());
                            Vector3d pos = pos1 + dir / 2.0;
                            publishText(pos, str, "world", "distance", getMessageID(DISTANCES, id++),
                                        COLOR(1, 0, 0, 0.5), 1, 0.02f);
                        }
                    }
                }
            }
        }
        if (msg.ids.size() > 0)
            sensor_location_pub.publish(msg);
    }
}

bool LighthouseEstimator::poseEstimationEPnP() {
    ros::Time start_time = ros::Time::now();
    while (!estimateSensorPositionsUsingRelativeDistances(0, calibrated_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 0, are the sensors visible to lighthouse 0?!");
        if ((ros::Time::now() - start_time).sec > 10) {
            ROS_WARN("time out");
            return false;
        }
    }

    epnp PnP;
    PnP.reset_correspondences();
    PnP.set_internal_parameters(0, 0, 1, 1);
    PnP.set_maximum_number_of_correspondences(calibrated_sensors.size());
    for (uint id:calibrated_sensors) {
        Vector3d pos, ray;
        Vector2d angles;
        sensors[id].get(0, pos);
        sensors[id].get(0, angles);
        rayFromLighthouseAngles(angles, ray);
        ROS_INFO("sensor %d\npos %f\t%f\t%f ray %f\t%f\t%f", id, pos[0], pos[1], pos[2],
                 ray[0], ray[1], ray[2]);
        Vector3d pos2(pos[0] / pos[1], 0, pos[2] / pos[1]);
        PnP.add_correspondence(pos[0], -pos[2], -pos[1], pos[0] / pos[1], -pos[2] / pos[1]);
        publishSphere(pos2, "world", "projected points", rand(), COLOR(0, 0, 1, 1), 0.05);
    }

    double R_est[3][3], t_est[3];
    double err2 = PnP.compute_pose(R_est, t_est);
    double rot_err, transl_err;

    PnP.print_pose(R_est, t_est);
    cout << "epnp error: " << err2 << endl;

    tf::Transform tf;
    tf.setOrigin(tf::Vector3(t_est[0], t_est[1], t_est[2]));
    tf.setRotation(tf::Quaternion(1, 0, 0, 0));
    publishTF(tf, "lighthouse1", "object_epnp");
}

bool LighthouseEstimator::poseEstimationP3P() {
    // TODO: include p3p properly
//    P3PEstimator p3PEstimator;
//    std::vector<Eigen::MatrixXd> result;
//    MatrixXd points2d(calibrated_sensors.size(), 2), points3d(calibrated_sensors.size(), 3);
//    uint i = 0;
//    for (uint id:calibrated_sensors) {
//        Vector3d pos, ray;
//        Vector2d angles;
//        sensors[id].get(0, pos);
//        sensors[id].get(0, angles);
//        rayFromLighthouseAngles(angles, ray);
//        points2d(i, 0) = ray(0) / ray(1);
//        points2d(i, 1) = ray(2) / ray(1);
//        points3d(i, 0) = pos(0);
//        points3d(i, 1) = pos(1);
//        points3d(i, 2) = pos(2);
//    }
//    result = p3PEstimator.estimate(points2d, points3d);
//    tf::Transform tf;
//    Vector3d position = result[0].topRightCorner(3, 1);
//    cout << "p3p: " << endl << result[0] << endl;
//    cout << "p3p position: " << endl << position << endl;
//    tf.setOrigin(tf::Vector3(position(0), position(1), position(2)));
//    tf.setRotation(tf::Quaternion(1, 0, 0, 0));
//    publishTF(tf, "lighthouse1", "object_p3p");
}

bool LighthouseEstimator::lighthousePoseEstimationLeastSquares() {
    ros::Time start_time = ros::Time::now();
    vector<int> visible_sensors;
    getVisibleCalibratedSensors(visible_sensors);

    if (visible_sensors.size() < 4)
        return false;

    while (!estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A, visible_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 0, are the sensors visible to lighthouse 0?!");
        if ((ros::Time::now() - start_time).sec > 10) {
            ROS_WARN("time out");
            return false;
        }
    }
    start_time = ros::Time::now();
    while (!estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B, visible_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 1, are the sensors visible to lighthouse 1?!");
        if ((ros::Time::now() - start_time).sec > 10) {
            ROS_WARN("time out");
            return false;
        }
    }

    PoseEstimatorSensorCloud::PoseEstimator estimator(visible_sensors.size());

    uint i = 0;
    for (int &sensor:visible_sensors) {
        Vector3d relLocation0, relLocation1;
        sensors[sensor].get(0, relLocation0);
        sensors[sensor].get(1, relLocation1);
        estimator.pos3D_A.block(0, i, 4, 1) << relLocation0(0), relLocation0(1), relLocation0(2), 1;
        estimator.pos3D_B.block(0, i, 4, 1) << relLocation1(0), relLocation1(1), relLocation1(2), 1;
        i++;
    }

    VectorXd pose(6);
    pose << 0, 0, 0, 0, 0, 0.001;

    Matrix4d RT_0, RT_1;
    getTransform(LIGHTHOUSE_A, "world", RT_0);
    getTransform(LIGHTHOUSE_B, "world", RT_1);
    estimator.pos3D_A = RT_0 * estimator.pos3D_A;
    estimator.pos3D_B = RT_1 * estimator.pos3D_B;

    NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double> *lm;
    numDiff = new NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>(estimator);
    lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double>(*numDiff);
    lm->parameters.maxfev = MAX_ITERATIONS;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);
    ROS_INFO("PoseEstimationSensorCloud finished after %ld iterations, with an error of %f", lm->iter, lm->fnorm);

    tf::Transform tf;
    getTFtransform(pose, tf);

    roboy_communication_middleware::LighthousePoseCorrection msg;
    msg.id = LIGHTHOUSE_B;
    msg.type = RELATIV;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);

}

bool LighthouseEstimator::objectPoseEstimationLeastSquares() {
    ros::Rate rate(10);
    ros::Time t0 = ros::Time::now(), t1;

    while (objectposeestimating) {
        t1 = ros::Time::now();

        vector<int> visible_sensors;
        getVisibleCalibratedSensors(visible_sensors);

        if (visible_sensors.size() < 4){
            ROS_INFO_THROTTLE(1, "object pose estimation aborted because only %ld sensors are visible", visible_sensors.size());
            continue;
        }

        PoseEstimatorSensorCloud::PoseEstimator estimator(visible_sensors.size());
        uint i = 0;
        for (auto sensor:visible_sensors) {
            Vector3d position3d, relLocation1;
            sensors[sensor].getPosition3D(position3d);
            sensors[sensor].getRelativeLocation(relLocation1);
            estimator.pos3D_A.block(0, i, 4, 1) << position3d(0), position3d(1), position3d(2), 1;
            estimator.pos3D_B.block(0, i, 4, 1) << relLocation1(0), relLocation1(1), relLocation1(2), 1;
            i++;
        }

        Matrix4d RT_0, RT_correct, RT_object;
        getTransform(LIGHTHOUSE_A, "world", RT_0);
        estimator.pos3D_B = RT_0 * estimator.pos3D_B;

//        static bool show = true;
//        if((int)(t1-t0).toSec()%3==0){
//            if(show) {
//                cout << "using sensors:\n";
//                int i = 0;
//                for (auto sensor:visible_sensors) {
//                    cout << sensor << endl;
//                    cout << estimator.pos3D_A.block(0, i, 4, 1) << endl;
////                    cout << estimator.pos3D_B.block(0, i, 4, 1) << endl;
//                    i++;
//                }
//                cout << endl;
//                show = false;
//            }
//        }else{
//            show = true;
//        }

        object_pose << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;

        NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator> *numDiff;
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double> *lm;
        numDiff = new NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>(estimator);
        lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorSensorCloud::PoseEstimator>, double>(*numDiff);
        lm->parameters.maxfev = MAX_ITERATIONS;
        lm->parameters.xtol = 1e-10;
        int ret = lm->minimize(object_pose);
        ROS_INFO_THROTTLE(1,
                          "object pose estimation using %ld sensors, finished after %ld iterations, with an error of %f",
                          visible_sensors.size(), lm->iter, lm->fnorm);

        getRTmatrix(RT_correct, object_pose);
        RT_object = RT_correct * RT_0;

        tf::Transform tf;
        getTFtransform(RT_object, tf);
        publishTF(tf, "world", "object");

        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        Quaterniond q;
        Vector3d origin;
        getPose(q,origin,object_pose);
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.position.x = origin(0);
        msg.pose.pose.position.y = origin(1);
        msg.pose.pose.position.z = origin(2);
        msg.pose.covariance = {
                0.1, 0, 0, 0, 0, 0,
                0, 0.1, 0, 0, 0, 0,
                0, 0, 0.1, 0, 0, 0,
                0, 0, 0, 0.1, 0, 0,
                0, 0, 0, 0, 0.1, 0,
                0, 0, 0, 0, 0, 0.1
        };
        pose_pub.publish(msg);

        publishMesh("darkroom","calibrated_objects/models", mesh.c_str(), origin, q, 0.001, "world", "mesh", 9999, 1);

//        if(lm->fnorm>0.1) // arbitrary but very bad
//            object_pose << 0, 0, 0, 0, 0, 0.1;
        rate.sleep();
    }
}

bool LighthouseEstimator::poseEstimationSensorDistance() {
    ros::Time start_time = ros::Time::now();
    int numberOfSamples = 0;
    ros::Rate rate(20);

    MatrixXd rays0_A(3, NUMBER_OF_SAMPLES), rays1_A(3, NUMBER_OF_SAMPLES),
            rays0_B(3, NUMBER_OF_SAMPLES), rays1_B(3, NUMBER_OF_SAMPLES);

    while (numberOfSamples < NUMBER_OF_SAMPLES) {
        Vector2d angles0, angles1;
        Vector3d ray0, ray1;

        // get the rays for the first sensor
        sensors[17].get(0, angles0);
        sensors[17].get(1, angles1);
        rayFromLighthouseAngles(angles0, ray0);
        rayFromLighthouseAngles(angles1, ray1);

        rays0_A(0, numberOfSamples) = ray0(0);
        rays0_A(1, numberOfSamples) = ray0(1);
        rays0_A(2, numberOfSamples) = ray0(2);

        rays0_B(0, numberOfSamples) = ray1(0);
        rays0_B(1, numberOfSamples) = ray1(1);
        rays0_B(2, numberOfSamples) = ray1(2);

        Vector3d zero(0, 0, 0);
        publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 1 + numberOfSamples, COLOR(0, 1, 0, 0.1));
        publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 2 + numberOfSamples, COLOR(0, 1, 0, 0.1));

        // get the rays for the second sensor
        sensors[18].get(0, angles0);
        sensors[18].get(1, angles1);
        rayFromLighthouseAngles(angles0, ray0);
        rayFromLighthouseAngles(angles1, ray1);

        rays1_A(0, numberOfSamples) = ray0(0);
        rays1_A(1, numberOfSamples) = ray0(1);
        rays1_A(2, numberOfSamples) = ray0(2);

        rays1_B(0, numberOfSamples) = ray1(0);
        rays1_B(1, numberOfSamples) = ray1(1);
        rays1_B(2, numberOfSamples) = ray1(2);

        publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 3 + numberOfSamples, COLOR(0, 0, 1, 0.1));
        publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 4 + numberOfSamples, COLOR(0, 0, 1, 0.1));

        numberOfSamples++;
        rate.sleep();
    }

    PoseEstimatorSensorDistance::PoseEstimator estimator(NUMBER_OF_SAMPLES, 0.237, rays0_A, rays0_B, rays1_A,
                                                         rays1_B);
    Matrix4d RT_0, RT_1;
    getTransform("world", LIGHTHOUSE_A, RT_0);
    getTransform("world", LIGHTHOUSE_B, RT_1);
    estimator.RT_A = RT_0;
    estimator.RT_B = RT_1;

    VectorXd pose(6);
    pose << 0, 0, 0, 0, 0, 0;

    NumericalDiff<PoseEstimatorSensorDistance::PoseEstimator> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorSensorDistance::PoseEstimator>, double> *lm;
    numDiff = new NumericalDiff<PoseEstimatorSensorDistance::PoseEstimator>(estimator);
    lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorSensorDistance::PoseEstimator>, double>(*numDiff);
    lm->parameters.maxfev = MAX_ITERATIONS;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);

    Eigen::Matrix<double, 3, 4> proj_matrix0, proj_matrix1;
    proj_matrix0 = RT_0.topLeftCorner(3, 4);

    Matrix4d RT_1_corrected = MatrixXd::Identity(4, 4);
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(pose(0), 2.0) + pow(pose(1), 2.0) + pow(pose(2), 2.0), 2.0);
    Quaterniond q((1 - alpha_squared) / (alpha_squared + 1),
                  2.0 * pose(0) / (alpha_squared + 1),
                  2.0 * pose(1) / (alpha_squared + 1),
                  2.0 * pose(2) / (alpha_squared + 1));
    q.normalize();
    // construct RT matrix
    RT_1_corrected.topLeftCorner(3, 3) = q.toRotationMatrix();
    RT_1_corrected.topRightCorner(3, 1) << pose(3), pose(4), pose(5);
    RT_1_corrected = RT_1_corrected * RT_1;
    proj_matrix1 = RT_1_corrected.topLeftCorner(3, 4);

//        cout << "fvec" << endl;
    for (int i = 0; i < numberOfSamples; i++) {
        // project onto image plane
        Eigen::Vector2d projected_image_location0 = Eigen::Vector2d(rays0_A(0, i) / rays0_A(2, i),
                                                                    rays0_A(1, i) / rays0_A(2, i));
        Eigen::Vector2d projected_image_location1 = Eigen::Vector2d(rays0_B(0, i) / rays0_B(2, i),
                                                                    rays0_B(1, i) / rays0_B(2, i));

        // TODO: use simpler trinagulation
//        Vector3d pos0 = triangulate_point(proj_matrix0, proj_matrix1,
//                                          projected_image_location0, projected_image_location1);

        projected_image_location0 = Eigen::Vector2d(rays1_A(0, i) / rays1_A(2, i),
                                                    rays1_A(1, i) / rays1_A(2, i));
        projected_image_location1 = Eigen::Vector2d(rays1_B(0, i) / rays1_B(2, i),
                                                    rays1_B(1, i) / rays1_B(2, i));

        // TODO: use simpler trinagulation
//        Vector3d pos1 = triangulate_point(proj_matrix0, proj_matrix1,
//                                          projected_image_location0, projected_image_location1);
//        publishSphere(pos0, "world", "tri", rand(), COLOR(1, 0, 0, 1));
//        publishSphere(pos1, "world", "tri", rand(), COLOR(1, 0, 0, 1));
//        ROS_INFO_STREAM((pos1 - pos0).norm());
    }


    tf::Transform tf;
    getTFtransform(pose, tf);

    roboy_communication_middleware::LighthousePoseCorrection msg;
    msg.id = LIGHTHOUSE_B;
    msg.type = ABSOLUT;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);

    ROS_INFO("pose estimation finished with %d", ret);
}

bool LighthouseEstimator::poseEstimationSensorDistances() {
    MatrixXd ray_A(4, 3), ray_B(4, 3);

    Vector2d angles0, angles1;
    Vector3d ray0, ray1;

    // get the rays for the first sensor
    sensors[16].get(0, angles0);
    sensors[16].get(1, angles1);
    rayFromLighthouseAngles(angles0, ray0);
    rayFromLighthouseAngles(angles1, ray1);

    ray_A(0, 0) = ray0(0);
    ray_A(1, 0) = ray0(1);
    ray_A(2, 0) = ray0(2);
    ray_A(3, 0) = 1;

    ray_B(0, 0) = ray1(0);
    ray_B(1, 0) = ray1(1);
    ray_B(2, 0) = ray1(2);
    ray_B(3, 0) = 1;

    Vector3d zero(0, 0, 0);
    publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 1, COLOR(1, 0, 0, 0.1));
    publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 2, COLOR(1, 0, 0, 0.1));

    // get the rays for the second sensor
    sensors[17].get(0, angles0);
    sensors[17].get(1, angles1);
    rayFromLighthouseAngles(angles0, ray0);
    rayFromLighthouseAngles(angles1, ray1);

    ray_A(0, 1) = ray0(0);
    ray_A(1, 1) = ray0(1);
    ray_A(2, 1) = ray0(2);
    ray_A(3, 1) = 1;

    ray_B(0, 1) = ray1(0);
    ray_B(1, 1) = ray1(1);
    ray_B(2, 1) = ray1(2);
    ray_B(3, 1) = 1;

    publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 3, COLOR(0, 1, 0, 0.1));
    publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 4, COLOR(0, 1, 0, 0.1));

    // get the rays for the third sensor
    sensors[18].get(0, angles0);
    sensors[18].get(1, angles1);
    rayFromLighthouseAngles(angles0, ray0);
    rayFromLighthouseAngles(angles1, ray1);

    ray_A(0, 2) = ray0(0);
    ray_A(1, 2) = ray0(1);
    ray_A(2, 2) = ray0(2);
    ray_A(3, 2) = 1;

    ray_B(0, 2) = ray1(0);
    ray_B(1, 2) = ray1(1);
    ray_B(2, 2) = ray1(2);
    ray_B(3, 2) = 1;

    publishRay(zero, ray0, "lighthouse1", "rays0", 9000 * 5, COLOR(0, 0, 1, 0.1));
    publishRay(zero, ray1, "lighthouse2", "rays1", 9000 * 6, COLOR(0, 0, 1, 0.1));

    PoseEstimatorSensorDistances::PoseEstimator estimator(3, ray_A, ray_B);

    VectorXd pose(6);
    pose << 0, 0, 0, 0, 0, -2;

    NumericalDiff<PoseEstimatorSensorDistances::PoseEstimator> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorSensorDistances::PoseEstimator>, double> *lm;
    numDiff = new NumericalDiff<PoseEstimatorSensorDistances::PoseEstimator>(estimator);
    lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorSensorDistances::PoseEstimator>, double>(*numDiff);
    lm->parameters.maxfev = 100;
    lm->parameters.xtol = 1.0e-10;
    int ret = lm->minimize(pose);

    tf::Transform tf;
    getTFtransform(pose, tf);

    roboy_communication_middleware::LighthousePoseCorrection msg;
    msg.id = LIGHTHOUSE_B;
    msg.type = ABSOLUT;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);

    ROS_INFO("pose estimation finished with %d", ret);
}

bool LighthouseEstimator::poseEstimationParticleFilter() {
    Matrix4d RT_0, RT_1;
    if (!getTransform(LIGHTHOUSE_A, "world", RT_0))
        return false;
    if (!getTransform(LIGHTHOUSE_B, "world", RT_1))
        return false;

    vector<int> ids;
    vector<Vector3d> rays0, rays1;
    Vector3d origin0, origin1;
    origin0 = RT_0.topRightCorner(3, 1);
    origin1 = RT_1.topRightCorner(3, 1);
    MatrixXd distances(calibrated_sensors.size(), calibrated_sensors.size());
    for (auto &sensor:calibrated_sensors) {
        if (sensors[sensor].isActive(0) && sensors[sensor].isActive(1)) {
            ids.push_back(sensor);
            Vector2d angles0, angles1;
            Vector3d ray0, ray1;
            sensors[sensor].get(0, angles0);
            sensors[sensor].get(1, angles1);
            rayFromLighthouseAngles(angles0, ray0);
            rayFromLighthouseAngles(angles1, ray1);
            Vector3d ray0_worldFrame, ray1_worldFrame;
            ray0_worldFrame = RT_0.topLeftCorner(3, 3) * ray0;
            ray1_worldFrame = RT_1.topLeftCorner(3, 3) * ray1;
            rays0.push_back(ray0_worldFrame);
            rays1.push_back(ray1_worldFrame);
        } else {
            ROS_ERROR("sensor %d of the calibrated sensors is not visible, aborting", sensor);
            return false;
        }
    }

    int i = 0;
    for (auto &sensor0:calibrated_sensors) {
        int j = 0;
        Vector3d rel0;
        sensors[sensor0].getRelativeLocation(rel0);
        for (auto &sensor1:calibrated_sensors) {
            if (sensor0 != sensor1) {
                Vector3d rel1;
                sensors[sensor1].getRelativeLocation(rel1);
                distances(i, j) = (rel0 - rel1).norm();
            }
            j++;
        }
        i++;
    }

    normal_distribution<double> distribution(0, 0.001);
    default_random_engine generator;

    VectorXd start(6);
    start << 0, 0, 0, RT_1(0, 3), RT_1(1, 3), RT_1(2, 3);

    {
        double distanceBetweenRays = 0.0;
        vector<Vector3d> triangulated_positions;
        for (uint sensor = 0; sensor < rays0.size(); sensor++) {
            Vector3d l0, l1;
            distanceBetweenRays += pow(dist3D_Line_to_Line(origin0, rays0[sensor], origin1,
                                                           rays1[sensor], l0, l1), 2.0);
            triangulated_positions.push_back(l0 + origin0 + (l1 + origin1 - l0 - origin0) / 2.0);
        }
        distanceBetweenRays = sqrt(distanceBetweenRays);

        double distanceBetweenSensors = 0.0;
        for (uint i = 0; i < rays0.size(); i++) {
            for (uint j = 0; j < rays1.size(); j++) {
                if (i != j) {
                    double dist = distances(i, j);
                    double triangulated_dist = (triangulated_positions[i] - triangulated_positions[j]).norm();
                    distanceBetweenSensors += pow(triangulated_dist - dist, 2.0);
                }
            }
        }
        distanceBetweenSensors = sqrt(distanceBetweenSensors);
        ROS_INFO("\nerror rays: %f\n"
                         "error sensor distances: %f", distanceBetweenRays, distanceBetweenSensors);
    }

    ParticleFilter<VectorXd> pf(NUMBER_OF_PARTICLES, 6,
                                [&pf, &distribution, &generator](int id) {
                                    double dx = distribution(generator), dy = distribution(
                                            generator), dz = distribution(generator);
                                    double droll = distribution(generator), dpitch = distribution(
                                            generator), dyaw = distribution(generator);
                                    pf.particles[id](0) += dx;
                                    pf.particles[id](1) += dy;
                                    pf.particles[id](2) += dz;
                                    pf.particles[id](3) += droll;
                                    pf.particles[id](4) += dpitch;
                                    pf.particles[id](5) += dyaw;
                                },
                                [this, &pf, &rays0, &rays1, &origin0, &distances](int id) {
                                    double distanceSensors = 0.0, distanceRays = 0.0;
                                    vector<Vector3d> triangulated_positions;
                                    Matrix4d RT = Matrix4d::Identity();
                                    getRTmatrix(RT, pf.particles[id]);
                                    for (uint sensor = 0; sensor < rays0.size(); sensor++) {
                                        Vector3d new_origin = RT.topRightCorner(3, 1);
                                        Vector3d new_ray = RT.topLeftCorner(3, 3) * rays1[sensor];
                                        Vector3d l0, l1;
                                        distanceSensors += pow(
                                                dist3D_Line_to_Line(origin0, rays0[sensor], new_origin,
                                                                    new_ray, l0, l1), 2.0);
                                        triangulated_positions.push_back(
                                                l0 + origin0 + (l1 + new_origin - l0 - origin0) / 2.0);
//                                        this->publishSphere(new_origin, "world", "particle",rand(), COLOR(1,0,0,1), 0.1);
//                                        this->publishRay(new_origin, new_ray, "world", "particle",rand(), COLOR(1,0,0,1), 0);
                                    }
                                    distanceSensors = sqrt(distanceSensors);
                                    for (uint i = 0; i < rays0.size(); i++) {
                                        for (uint j = 0; j < rays1.size(); j++) {
                                            if (i != j) {
                                                double dist = distances(i, j);
                                                double triangulated_dist = (triangulated_positions[i] -
                                                                            triangulated_positions[j]).norm();
                                                distanceRays += pow(triangulated_dist - dist, 2.0);
                                            }
                                        }
                                    }
                                    distanceRays = sqrt(distanceRays);

                                    return (distanceRays + distanceSensors);
                                },
                                start, 0.001);
    uint iter = 0;
    while (iter < 1000 && particle_filtering) {
        pf.step();
        ROS_INFO("particle iterations %d", iter);
        for (uint i = 0; i < NUMBER_OF_PARTICLES; i++) {
            tf::Transform tf;
            getTFtransform(pf.particles[i], tf);
            char str[100];
            sprintf(str, "%d", i);
            publishTF(tf, "world", str);
        }
        iter++;
    }

    VectorXd winner;
    int index;
    double cost = pf.result(winner, &index);


    getRTmatrix(RT_1, pf.particles[index]);
    origin1 = RT_1.topRightCorner(3, 1);

    double distanceBetweenRays = 0.0;
    vector<Vector3d> triangulated_positions;
    for (uint sensor = 0; sensor < rays0.size(); sensor++) {
        Vector3d l0, l1;
        rays1[sensor] = RT_1.topRightCorner(3, 3) * rays1[sensor];
        distanceBetweenRays += pow(dist3D_Line_to_Line(origin0, rays0[sensor], origin1,
                                                       rays1[sensor], l0, l1), 2.0);
        triangulated_positions.push_back(l0 + origin0 + (l1 + origin1 - l0 - origin0) / 2.0);
    }
    distanceBetweenRays = sqrt(distanceBetweenRays);

    double distanceBetweenSensors = 0.0;
    for (uint i = 0; i < rays0.size(); i++) {
        for (uint j = 0; j < rays1.size(); j++) {
            if (i != j) {
                double dist = distances(i, j);
                double triangulated_dist = (triangulated_positions[i] - triangulated_positions[j]).norm();
                distanceBetweenSensors += pow(triangulated_dist - dist, 2.0);
            }
        }
    }
    distanceBetweenSensors = sqrt(distanceBetweenSensors);

    ROS_INFO("particle filter terminated with %f cost and the winner is %d\n"
                     "rot:\n"
                     "%f\t%f\t%f\n"
                     "%f\t%f\t%f\n"
                     "%f\t%f\t%f\n"
                     "trans: %f\t%f\t%f\n"
                     "error rays %f\n"
                     "error distance between sensors %f", cost, index,
             RT_1(0, 0), RT_1(0, 1), RT_1(0, 2), RT_1(1, 0), RT_1(1, 1), RT_1(1, 2), RT_1(2, 0),
             RT_1(2, 1), RT_1(2, 2), RT_1(0, 3), RT_1(1, 3), RT_1(2, 3), distanceBetweenRays, distanceBetweenSensors);


    tf::Transform tf;
    tf.setBasis(tf::Matrix3x3(RT_1(0, 0), RT_1(0, 1), RT_1(0, 2), RT_1(1, 0), RT_1(1, 1), RT_1(1, 2), RT_1(2, 0),
                              RT_1(2, 1), RT_1(2, 2)));
    tf.setOrigin(tf::Vector3(RT_1(0, 3), RT_1(1, 3), RT_1(2, 3)));

    roboy_communication_middleware::LighthousePoseCorrection msg;
    msg.id = LIGHTHOUSE_B;
    msg.type = ABSOLUT;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);
}

int LighthouseEstimator::getMessageID(int type, int sensor, bool lighthouse) {
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
