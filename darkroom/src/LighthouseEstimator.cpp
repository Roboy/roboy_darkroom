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
    ootx_sub = nh->subscribe("/roboy/middleware/DarkRoom/ootx", 1, &LighthouseEstimator::receiveOOTXData, this);
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

void LighthouseEstimator::objectPoseEstimationLeastSquares() {
    ros::Rate rate(30);
    ros::Time t0 = ros::Time::now(), t1;

    pose_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_name.c_str(), 1);

    object_pose << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;

    while (objectposeestimating) {
        t1 = ros::Time::now();

        vector<int> visible_sensors;
        getVisibleCalibratedSensors(visible_sensors);

        if (visible_sensors.size() < 4) {
            ROS_INFO_THROTTLE(1, "object pose estimation aborted because only %ld sensors are visible",
                              visible_sensors.size());
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

//        object_pose << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;

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

        if (lm->fnorm > 0.001) {
            object_pose << 0, 0, 0, 0, 0, 0.1;
        } else {
            getRTmatrix(RT_correct, object_pose);
            RT_object = RT_correct * RT_0;

            tf::Transform tf;
            getTFtransform(RT_object, tf);
            string tf_name = name + "_VO";
            publishTF(tf, "world", tf_name.c_str());

            geometry_msgs::PoseWithCovarianceStamped msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "world";
            Quaterniond q;
            Vector3d origin;
            getPose(q, origin, object_pose);
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

            if (has_mesh) // TODO mesh path not properly implemented yet
                publishMesh("roboy_models", "Roboy2.0_Upper_Body_Xylophone_simplified/meshes/CAD", "xylophone.stl",
                            origin, q, 0.001, "world", "mesh", 9999, 1);
        }
//        rate.sleep();
    }

    pose_pub.shutdown();
}

bool LighthouseEstimator::estimateSensorPositionsUsingRelativeDistances(bool lighthouse, vector<int> &specificIds) {
    ROS_INFO_STREAM("estimating distance of sensors to lighthouse " << lighthouse + 1);
    vector <Vector3d> relPos;
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
        if (error < ERROR_THRESHOLD || (error - error_prev) < 0.00000001) {
            break;
        }
        error_prev = error;
        ROS_INFO_THROTTLE(5, "iteration %d error %lf", iterations, error);
        // construct distance new vector, sharing data with the stl container
        Map <VectorXd> d_new(distanceToLighthouse.data(), distanceToLighthouse.size());
        d_new = d_old - (J.transpose() * J).inverse() * J.transpose() * v;
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

bool LighthouseEstimator::estimateObjectPoseUsingRelativeDistances() {
    vector<int> visible_sensors[2];
    getVisibleCalibratedSensors(LIGHTHOUSE_A, visible_sensors[LIGHTHOUSE_A]);
    VectorXd pose[2];
    Matrix4d RT_object[2];
    if (visible_sensors[LIGHTHOUSE_A].size() >= 4) {
        if (estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A, visible_sensors[LIGHTHOUSE_A])) {
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
            publishTF(tf, (LIGHTHOUSE_A ? "lighthouse2" : "lighthouse1"), "object_lighthouse_1");
        }
    }

    getVisibleCalibratedSensors(LIGHTHOUSE_B, visible_sensors[LIGHTHOUSE_B]);
    if (visible_sensors[LIGHTHOUSE_B].size() >= 4) {
        if (estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B, visible_sensors[LIGHTHOUSE_B])) {
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
            publishTF(tf, (LIGHTHOUSE_B ? "lighthouse2" : "lighthouse1"), "object_lighthouse_2");
        }
    }

    Matrix4d RT_correct = RT_object[LIGHTHOUSE_A] * RT_object[LIGHTHOUSE_B].inverse();

    tf::Transform tf;
    tf.setOrigin(tf::Vector3(RT_correct(0, 3), RT_correct(1, 3), RT_correct(2, 3)));
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
    map < int, high_resolution_clock::time_point[4] > timestamps_old;

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

        int active_sensors_counter = 0;

        for (auto &sensor : sensors) {
            lighthouse_active[LIGHTHOUSE_A] = sensor.second.isActive(LIGHTHOUSE_A);
            lighthouse_active[LIGHTHOUSE_B] = sensor.second.isActive(LIGHTHOUSE_B);
            if (sensor.second.hasNewData(timestamps_old[sensor.first])) {
                Vector2d lighthouse0_angles;
                Vector2d lighthouse1_angles;
                sensor.second.get(LIGHTHOUSE_A, lighthouse0_angles, &timestamp_new[LIGHTHOUSE_A * 2]);
                sensor.second.get(LIGHTHOUSE_B, lighthouse1_angles, &timestamp_new[LIGHTHOUSE_B * 2]);

                memcpy(timestamps_old[sensor.first], timestamp_new, sizeof(timestamp_new));

                Vector3d ray0, ray1;

                if (lighthouse_active[LIGHTHOUSE_A] && lighthouse_active[LIGHTHOUSE_B]) {
                    active_sensors_counter++;

                    Vector3d triangulated_position;

                    applyCalibrationData(lighthouse0_angles, lighthouse1_angles);

                    triangulateFromLighthouseAngles(lighthouse0_angles, lighthouse1_angles, RT_0, RT_1,
                                                    triangulated_position, ray0,
                                                    ray1);

                    sensor.second.set(triangulated_position);

                    if (!triangulated_position.hasNaN()) {
                        char str[100], str2[2];
                        sprintf(str, "sensor_%d", sensor.first);
                        publishSphere(triangulated_position, "world", str,
                                      getMessageID(TRIANGULATED, sensor.first), COLOR(0, 1, 0, 0.8), 0.01f, 1);
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
                                   COLOR(0, 1, 0, 1.0), 1);
                        ray1 *= 5;
                        publishRay(pos, ray1, "lighthouse2", "rays_lighthouse_2", getMessageID(RAY, sensor.first, 1),
                                   COLOR(0, 1, 0, 1.0), 1);

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
        active_sensors = active_sensors_counter;
        if (msg.ids.size() > 0)
            sensor_location_pub.publish(msg);
    }
}

void LighthouseEstimator::publishRays() {
    high_resolution_clock::time_point timestamp_new[4];
    map < int, high_resolution_clock::time_point[4] > timestamps_old;

    ros::Rate rate(30);
    bool lighthouse_active[2];

    while (rays) {
        for (auto &sensor : sensors) {
            lighthouse_active[LIGHTHOUSE_A] = sensor.second.isActive(LIGHTHOUSE_A);
            lighthouse_active[LIGHTHOUSE_B] = sensor.second.isActive(LIGHTHOUSE_B);
            if (sensor.second.hasNewData(timestamps_old[sensor.first])) {
                Vector2d lighthouse0_angles;
                Vector2d lighthouse1_angles;
                sensor.second.get(LIGHTHOUSE_A, lighthouse0_angles, &timestamp_new[LIGHTHOUSE_A * 2]);
                sensor.second.get(LIGHTHOUSE_B, lighthouse1_angles, &timestamp_new[LIGHTHOUSE_B * 2]);

                memcpy(timestamps_old[sensor.first], timestamp_new, sizeof(timestamp_new));

                if (lighthouse_active[LIGHTHOUSE_A]) {
                    Vector3d ray;
                    rayFromLighthouseAngles(lighthouse0_angles, ray, LIGHTHOUSE_A);
                    Vector3d pos(0, 0, 0);
                    ray *= 5;
                    publishRay(pos, ray, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
                               COLOR(1, 0, 0, 0.5), 1);
                }
                if (lighthouse_active[LIGHTHOUSE_B]) {
                    Vector3d ray;
                    rayFromLighthouseAngles(lighthouse1_angles, ray, LIGHTHOUSE_B);
                    Vector3d pos(0, 0, 0);
                    ray *= 5;
                    publishRay(pos, ray, "lighthouse2", "rays_lighthouse_2", getMessageID(RAY, sensor.first, 0),
                               COLOR(1, 0, 0, 0.5), 1);
                }
            }
        }
        rate.sleep();
    }
}

void LighthouseEstimator::calibrateRelativeSensorDistances() {
    map<int, vector<Vector3d>> sensorPosition3d;
    map<int, int> number_of_samples;

    // measure the sensor location for a couple of seconds
    ros::Time start_time = ros::Time::now();
    clearAll();

    high_resolution_clock::time_point timestamp0_new[2], timestamp1_new[2];
    map < int, high_resolution_clock::time_point[2] > timestamps0_old, timestamps1_old;

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

bool LighthouseEstimator::estimateFactoryCalibration(int lighthouse) {
    vector<double> elevation_measured, azimuth_measured, elevation_truth, azimuth_truth;
    stringstream str;
    // get the transform from object to lighthouse
    Matrix4d RT_object2lighthouse;
    ros::Time t0 = ros::Time::now();
    while (!getTransform(name.c_str(), (lighthouse ? "lighthouse2" : "lighthouse1"), RT_object2lighthouse)) {
        ROS_ERROR_THROTTLE(1, "could not get transform between %s and %s", name.c_str(),
                           (lighthouse ? "lighthouse2" : "lighthouse1"));
        if ((ros::Duration(ros::Time::now() - t0).toSec() > 5)) {
            ROS_ERROR("giving up");
            return false;
        }
    }
    ROS_INFO("got the transform, let's see who is active");

    // lets see who is active
    vector<int> active_sensors;
    t0 = ros::Time::now();
    while (active_sensors.size() < 20) {
        getVisibleCalibratedSensors(lighthouse, active_sensors);
        if (active_sensors.size() < 20 && (ros::Duration(ros::Time::now() - t0).toSec() > 5)) {
            ROS_ERROR("no active sensors, giving up");
            return false;
        }
    }
    ROS_INFO("%ld active sensors", active_sensors.size());

    for (auto &active_sensor:active_sensors) {
        // calculate the ground truth
        Vector4d rel_pos;
        sensors[active_sensor].getRelativeLocation(rel_pos);
        // transform into lighthouse frame
        Vector4d pos_in_lighthouse_frame = RT_object2lighthouse * rel_pos;

        double distance = sqrt(pow(pos_in_lighthouse_frame[0], 2.0) + pow(pos_in_lighthouse_frame[1], 2.0) +
                               pow(pos_in_lighthouse_frame[2], 2.0));

        str << active_sensor << " sensor position in ligthhouse frame: " << pos_in_lighthouse_frame[0] << " "
            << pos_in_lighthouse_frame[1] << " " << pos_in_lighthouse_frame[2] << endl;

        // calculate ground truth ligthhouse angles
        double elevation = M_PI - acos(pos_in_lighthouse_frame[2] / distance);
        double azimuth = atan2(pos_in_lighthouse_frame[1], pos_in_lighthouse_frame[0]);
        vector<double> a = {M_PI - elevation, azimuth};
        elevation_truth.push_back(M_PI - elevation);
        azimuth_truth.push_back(azimuth);

        // get the measured angles
        sensors[active_sensor].get(lighthouse, elevation, azimuth);
        elevation_measured.push_back(elevation);
        azimuth_measured.push_back(azimuth);

        str << "\t" << elevation_truth.back() << "\t"
            << elevation_measured.back() << "\t"
            << "\t" << azimuth_truth.back() << "\t" << azimuth_measured.back() << endl;
    }
    ROS_INFO_STREAM(str.str());

    InYourGibbousPhase::InYourGibbousPhase estimator(elevation_measured.size());
    estimator.elevation_measured = elevation_measured;
    estimator.azimuth_measured = azimuth_measured;
    estimator.elevation_truth = elevation_truth;
    estimator.azimuth_truth = azimuth_truth;

    NumericalDiff<InYourGibbousPhase::InYourGibbousPhase> *numDiff1;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<InYourGibbousPhase::InYourGibbousPhase>, double> *lm;
    numDiff1 = new NumericalDiff<InYourGibbousPhase::InYourGibbousPhase>(estimator);
    lm = new LevenbergMarquardt<NumericalDiff<InYourGibbousPhase::InYourGibbousPhase>, double>(*numDiff1);
    lm->parameters.maxfev = 100000;
    lm->parameters.xtol = 1e-10;
    VectorXd calibrationValues(10);
    calibrationValues << 0.0000001, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    int ret = lm->minimize(calibrationValues);

    ROS_INFO_STREAM("calibration value estimation for lighthouse " << lighthouse + 1 << " terminated with error "
                                                                   << lm->fnorm << endl
                                                                   << "phase vertical    " << calibrationValues(phase0)
                                                                   << endl
                                                                   << "phase horizontal      " << calibrationValues(phase1)
                                                                   << endl
                                                                   << "tilt vertical     " << calibrationValues(tilt0)
                                                                   << endl
                                                                   << "tilt horizontal       " << calibrationValues(tilt1)
                                                                   << endl
                                                                   << "curve vertical    " << calibrationValues(curve0)
                                                                   << endl
                                                                   << "curve horizontal      " << calibrationValues(curve1)
                                                                   << endl
                                                                   << "gibphase vertical " << calibrationValues(gibphase0)
                                                                   << endl
                                                                   << "gibphase horizontal   " << calibrationValues(gibphase1)
                                                                   << endl
                                                                   << "gibmag vertical   " << calibrationValues(gibmag0)
                                                                   << endl
                                                                   << "gibmag horizontal     " << calibrationValues(gibmag1));
    calibration[lighthouse][VERTICAL].phase = calibrationValues(phase0);
    calibration[lighthouse][HORIZONTAL].phase = calibrationValues(phase1);
    calibration[lighthouse][VERTICAL].tilt = calibrationValues(tilt0);
    calibration[lighthouse][HORIZONTAL].tilt = calibrationValues(tilt1);
    calibration[lighthouse][VERTICAL].curve = calibrationValues(curve0);
    calibration[lighthouse][HORIZONTAL].curve = calibrationValues(curve1);
    calibration[lighthouse][VERTICAL].gibphase = calibrationValues(gibphase0);
    calibration[lighthouse][HORIZONTAL].gibphase = calibrationValues(gibphase1);
    calibration[lighthouse][VERTICAL].gibmag = calibrationValues(gibmag0);
    calibration[lighthouse][HORIZONTAL].gibmag = calibrationValues(gibmag1);

    string package_path = ros::package::getPath("darkroom");
    string calibration_result_path = package_path + "/params/lighthouse_calibration.yaml";

    return writeCalibrationConfig(calibration_result_path, lighthouse, calibration[lighthouse]);
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

void LighthouseEstimator::receiveOOTXData(const roboy_communication_middleware::DarkRoomOOTX::ConstPtr &msg) {
    ootx[msg->lighthouse].fw_version = msg->fw_version;
    ootx[msg->lighthouse].ID = msg->ID;
    ootx[msg->lighthouse].fcal_0_phase = msg->fcal_0_phase;
    ootx[msg->lighthouse].fcal_1_phase = msg->fcal_1_phase;
    ootx[msg->lighthouse].fcal_0_tilt = msg->fcal_0_tilt;
    ootx[msg->lighthouse].fcal_1_tilt = msg->fcal_1_tilt;
    ootx[msg->lighthouse].unlock_count = msg->unlock_count;
    ootx[msg->lighthouse].hw_version = msg->hw_version;
    ootx[msg->lighthouse].fcal_0_curve = msg->fcal_0_curve;
    ootx[msg->lighthouse].fcal_1_curve = msg->fcal_1_curve;
    ootx[msg->lighthouse].accel_dir_x = msg->accel_dir_x;
    ootx[msg->lighthouse].accel_dir_y = msg->accel_dir_y;
    ootx[msg->lighthouse].accel_dir_z = msg->accel_dir_z;
    ootx[msg->lighthouse].fcal_0_gibphase = msg->fcal_0_gibphase;
    ootx[msg->lighthouse].fcal_1_gibphase = msg->fcal_1_gibphase;
    ootx[msg->lighthouse].fcal_0_gibmag = msg->fcal_0_gibmag;
    ootx[msg->lighthouse].fcal_1_gibmag = msg->fcal_1_gibmag;
    ootx[msg->lighthouse].mode = msg->mode;
    ootx[msg->lighthouse].faults = msg->faults;
}

void LighthouseEstimator::applyCalibrationData(Vector2d &lighthouse0_angles, Vector2d &lighthouse1_angles) {
    applyCalibrationData(LIGHTHOUSE_A, lighthouse0_angles);
    applyCalibrationData(LIGHTHOUSE_B, lighthouse1_angles);
}

void LighthouseEstimator::applyCalibrationData(bool lighthouse, Vector2d &lighthouse_angles) {
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

void LighthouseEstimator::applyCalibrationData(bool lighthouse, double &elevation, double &azimuth) {
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
