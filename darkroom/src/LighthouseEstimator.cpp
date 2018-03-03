#include <common_utilities/CommonDefinitions.h>
#include "darkroom/LighthouseEstimator.hpp"

vector<vector<Vector3d>> relative_positions_trajectory;
vector<vector<Vector2d>> angles_measured_trajectory;
int numberOfSensors;
vector<double> elevation_measured, azimuth_measured, elevation_truth, azimuth_truth;

void function1_fvec(const real_1d_array &x, real_1d_array &fi, void *ptr) {
    double elevation, azimuth;
    for (int i = 0; i < relative_positions_trajectory.size(); i++) {
        double error_elevation = 0, error_azimuth = 0;
        for (int j = 0; j < angles_measured_trajectory[i].size(); j++) {
            elevation = angles_measured_trajectory[i][j](0), azimuth = angles_measured_trajectory[i][j](1);
            double elevation_truth = M_PI - atan2(2.0, relative_positions_trajectory[i][j](2) + x[i * 2 + 1]);
            double azimuth_truth = atan2(2.0, relative_positions_trajectory[i][j](0) + x[i * 2]);

            error_elevation += (elevation - elevation_truth);
            error_azimuth += (azimuth - azimuth_truth);

        }
        fi[(i * 2)] = error_elevation;
        fi[(i * 2) + 1] = error_azimuth;
    }
}

void inYourGibbousPhase(const real_1d_array &x, real_1d_array &fi, void *ptr) {
    double elevation, azimuth;
    fi[0] = 0;
    fi[1] = 0;
    for (int i = 0; i < numberOfSensors; i++) {
        elevation = elevation_measured[i], azimuth = azimuth_measured[i];
        elevation += x[phase_vertical];
        azimuth += x[phase_horizontal];
        elevation +=
                x[curve_vertical] * pow(cos(azimuth), 2.0) + x[gibmag_vertical] * cos(elevation + x[gibphase_vertical]);
        azimuth += x[curve_horizontal] * pow(cos(elevation), 2.0) +
                   x[gibmag_horizontal] * cos(azimuth + x[gibphase_horizontal]);

        fi[0] += (elevation - elevation_truth[i]);
        fi[1] += (azimuth - azimuth_truth[i]);
//            cout << elevation << "\t" << azimuth << endl;
    }
}

//void function1_fvec(const real_1d_array &x, double &fi, void *ptr) {
//    double elevation, azimuth;
//    fi = 0;
//    for (int i = 0; i < relative_positions_trajectory.size(); i++) {
//        double error_elevation = 0, error_azimuth = 0;
//        for (int j = 0; j < angles_measured_trajectory[i].size(); j++) {
//            elevation = angles_measured_trajectory[i][j](0), azimuth = angles_measured_trajectory[i][j](1);
//            elevation += x[phase_vertical];
//            azimuth += x[phase_horizontal];
//
//            elevation += x[curve_vertical] * pow(sin(elevation) * cos(azimuth), 2.0) +
//                         x[gibmag_vertical] * cos(elevation + x[gibphase_vertical]);
//
//            azimuth += x[curve_horizontal] * pow(cos(elevation), 2.0) +
//                       x[gibmag_horizontal] * cos(azimuth + x[gibphase_horizontal]);
//
//            double elevation_truth = M_PI - atan2(2.0, relative_positions_trajectory[i][j](2) + x[10 + i * 2 + 1]);
//            double azimuth_truth = atan2(2.0, relative_positions_trajectory[i][j](0) + x[10 + i * 2]);
//
//            error_elevation += (elevation - elevation_truth);
//            error_azimuth += (azimuth - azimuth_truth);
//
//        }
//        fi += error_elevation;
//        fi += error_azimuth;
//    }
//}

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

    if (visible_sensors.size() < 4) {
        ROS_ERROR("we need at least four visible sensors for this to work, aborting...");
        return false;
    }

    while (!estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_A, visible_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 0, are the sensors visible to lighthouse 0?!");
        if (ros::Duration(ros::Time::now() - start_time).toSec() > 3) {
            ROS_WARN("time out");
            return false;
        }
    }
    start_time = ros::Time::now();
    while (!estimateSensorPositionsUsingRelativeDistances(LIGHTHOUSE_B, visible_sensors)) {
        ROS_INFO_THROTTLE(1,
                          "could not estimate relative distance to lighthouse 1, are the sensors visible to lighthouse 1?!");
        if (ros::Duration(ros::Time::now() - start_time).toSec() > 3) {
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
    ros::Rate rate(60);
    ros::Time t0 = ros::Time::now(), t1;

    pose_pub = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(pose_topic_name.c_str(), 1);

    object_pose << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;

    while (objectposeestimating) {
        t1 = ros::Time::now();

        vector<int> visible_sensors;
        getVisibleCalibratedSensors(visible_sensors);

        if (visible_sensors.size() < 4) {
            ROS_INFO_THROTTLE(1, "object pose estimation aborted because only %ld sensors are visible (need minimum 4)",
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

        if (lm->fnorm > 0.1) {
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

            if (comparesteamvr && steamVRrecord.is_open()) {
                tf::Transform frame;
                if (getTransform("world", "vive_controller1", frame)) {
                    tf::Vector3 origin2 = frame.getOrigin();
                    tf::Quaternion q2 = frame.getRotation();
                    steamVRrecord << ros::Time::now().toNSec() << ",\t"
                                  << origin(0) << ",\t" << origin(1) << ",\t" << origin(2) << ",\t"
                                  << q.x() << ",\t" << q.y() << ",\t" << q.z() << ",\t" << q.w() << ",\t"
                                  << origin2.x() << ",\t" << origin2.y() << ",\t" << origin2.z() << ",\t"
                                  << q2.x() << ",\t" << q2.y() << ",\t" << q2.z() << ",\t" << q2.w() << endl;
                }
            }
        }
        rate.sleep();
    }

    pose_pub.shutdown();
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
                // apply factory calibration correction
                applyCalibrationData(lighthouse, elevations.back(), azimuths.back());
                sensor.second.getRelativeLocation(relPos);
                distanceToLighthouse.push_back(sensor.second.getDistance(lighthouse));
//                distanceToLighthouse.push_back(rand()/(double)RAND_MAX);
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
            // apply factory calibration correction
            applyCalibrationData(lighthouse, elevations.back(), azimuths.back());
            sensors[specificIds.at(i)].getRelativeLocation(relPos);
            distanceToLighthouse.push_back(sensors[specificIds.at(i)].getDistance(lighthouse));
//            distanceToLighthouse.push_back(rand()/(double)RAND_MAX);
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
//            cosineBetween(i, j) =
//                    sin(elevations[i]) * cos(azimuths[i]) * sin(elevations[j]) * cos(azimuths[j]) +
//                    sin(elevations[i]) * sin(azimuths[i]) * sin(elevations[j]) * sin(azimuths[j]) +
//                    cos(elevations[i]) * cos(elevations[j]);
            double norm_1 = sqrt(pow(cos(azimuths[i]) * sin(elevations[i]), 2.0)
                                 + pow(sin(azimuths[i]) * sin(elevations[i]), 2.0)
                                 + pow(sin(azimuths[i]) * cos(elevations[i]), 2.0));
            double norm_2 = sqrt(pow(cos(azimuths[j]) * sin(elevations[j]), 2.0)
                                 + pow(sin(azimuths[j]) * sin(elevations[j]), 2.0)
                                 + pow(sin(azimuths[j]) * cos(elevations[j]), 2.0));
            cosineBetween(i, j) =
                    (cos(azimuths[i]) * sin(elevations[i]) * cos(azimuths[j]) * sin(elevations[j]) +
                     sin(azimuths[i]) * sin(elevations[i]) * sin(azimuths[j]) * sin(elevations[j]) +
                     sin(azimuths[i]) * cos(elevations[i]) * sin(azimuths[j]) * cos(elevations[j])) / (norm_1 * norm_2);

//            ROS_DEBUG("cosine between %d and %d: %f", i, j, cosineBetween(i, j));
            // calculate the distance between the sensors
            distanceBetween(i, j) = (relPos[i] - relPos[j]).norm();
//            ROS_DEBUG("distance between sensor %d and %d: %f", ids[i], ids[j], distanceBetween(i, j));
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
        MatrixXd J_pinv = Pinv(J);
        d_new = d_old - J_pinv * v;
        iterations++;
    }

    uint i = 0;
    for (auto id:ids) {
        ROS_DEBUG_STREAM("sensor:" << id << " distance to lighthouse " << lighthouse << ": " << d_old(i));

        Vector2d angles(azimuths[i], elevations[i]);
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
    } else {
        ROS_ERROR("we need at least four sensors for to be visble for lighthouse 1 for this to work, "
                          "but there are only %ld sensors visible aborting", visible_sensors[LIGHTHOUSE_A].size());
        return false;
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
    } else {
        ROS_ERROR("we need at least four sensors for to be visble for lighthouse 2 for this to work, "
                          "but there are only %ld sensors visible aborting", visible_sensors[LIGHTHOUSE_B].size());
        return false;
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
    msg.type = RELATIV;
    tf::transformTFToMsg(tf, msg.tf);
    lighthouse_pose_correction.publish(msg);
}

void LighthouseEstimator::estimateObjectPoseEPNP() {
    ros::Rate rate(30);
    Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
    while (poseestimating_epnp) {
        vector<int> visible_sensors[2];
        getVisibleCalibratedSensors(LIGHTHOUSE_A, visible_sensors[LIGHTHOUSE_A]);
        Matrix4d RT_object2lighthouse_true;

        if (visible_sensors[LIGHTHOUSE_A].size() >= 4) {
            epnp PnP;
            PnP.set_internal_parameters(0, 0, 1, 1);
            PnP.set_maximum_number_of_correspondences(visible_sensors[LIGHTHOUSE_A].size());
            PnP.reset_correspondences();
            for (int sensor:visible_sensors[LIGHTHOUSE_A]) {
                Vector3d rel_pos;
                sensors[sensor].getRelativeLocation(rel_pos);
                Vector2d angles;
                sensors[sensor].get(LIGHTHOUSE_A, angles);
                PnP.add_correspondence(rel_pos[2], rel_pos[1], rel_pos[0], tan(M_PI_2 - angles[HORIZONTAL]),
                                       tan(M_PI_2 - angles[VERTICAL]));
            }

            double R_est[3][3], t_est[3], R_true[3][3], t_true[3];
            double err2 = PnP.compute_pose(R_est, t_est);

            Matrix4d RT_object2lighthouse_est, RT_object2lighthouse_est_backup;

            RT_object2lighthouse_est.setIdentity();
            RT_object2lighthouse_est.block(0, 0, 3, 3) = Map<Matrix3d>(&R_est[0][0], 3, 3);
            RT_object2lighthouse_est.block(0, 3, 3, 1) = Map<Vector3d>(&t_est[0], 3, 1);
            RT_object2lighthouse_est_backup = RT_object2lighthouse_est;
            RT_object2lighthouse_est.block(0, 0, 3, 1) = -1.0 * RT_object2lighthouse_est_backup.block(0, 1, 3, 1);
            RT_object2lighthouse_est.block(0, 2, 3, 1) = RT_object2lighthouse_est_backup.block(0, 0, 3, 1);
            RT_object2lighthouse_est.block(0, 1, 3, 1) = -1.0 * RT_object2lighthouse_est_backup.block(0, 2, 3, 1);
            RT_object2lighthouse_est.block(0, 3, 3, 1) << RT_object2lighthouse_est_backup(0, 3),
                    RT_object2lighthouse_est_backup(2, 3),
                    -RT_object2lighthouse_est_backup(1, 3);

            ROS_INFO_STREAM_THROTTLE(5, RT_object2lighthouse_est.format(fmt) << endl
                                                                             << RT_object2lighthouse_true.format(fmt));

            tf::Transform tf;
            getTFtransform(RT_object2lighthouse_est, tf);
            string tf_name = name + "_epnp_1";
            publishTF(tf, "lighthouse1", tf_name.c_str());

            // this is the true pose (only available when simulating of course)
            if (getTransform(name.c_str(), "lighthouse1", RT_object2lighthouse_true)) {
                double rot_err, transl_err;
                Map<Matrix3d>(&R_true[0][0], 3, 3) = RT_object2lighthouse_true.block(0, 0, 3, 3);
                Map<Vector3d>(&t_true[0], 3, 1) = RT_object2lighthouse_true.block(0, 3, 3, 1);
                PnP.relative_error(rot_err, transl_err, R_true, t_true, R_est, t_est);
                stringstream str;
                str << ">>> Reprojection error: " << err2 << endl;
                str << ">>> rot_err: " << rot_err << ", transl_err: " << transl_err << endl;
                str << endl;
                str << "'True reprojection error':"
                    << PnP.reprojection_error(R_true, t_true) << endl;
                ROS_INFO_STREAM_THROTTLE(5, str.str());
            }
        } else {
            ROS_ERROR_THROTTLE(5, "we need at least four sensors for to be visible for lighthouse 1 for this to work, "
                    "but there are only %ld sensors visible aborting", visible_sensors[LIGHTHOUSE_A].size());
        }

        getVisibleCalibratedSensors(LIGHTHOUSE_B, visible_sensors[LIGHTHOUSE_B]);

        if (visible_sensors[LIGHTHOUSE_B].size() >= 4) {
            epnp PnP;
            PnP.set_internal_parameters(0, 0, 1, 1);
            PnP.set_maximum_number_of_correspondences(visible_sensors[LIGHTHOUSE_B].size());
            PnP.reset_correspondences();
            for (int sensor:visible_sensors[LIGHTHOUSE_B]) {
                Vector3d rel_pos;
                sensors[sensor].getRelativeLocation(rel_pos);
                Vector2d angles;
                sensors[sensor].get(LIGHTHOUSE_B, angles);
                PnP.add_correspondence(rel_pos[2], rel_pos[1], rel_pos[0], tan(M_PI_2 - angles[HORIZONTAL]),
                                       tan(M_PI_2 - angles[VERTICAL]));
            }

            double R_est[3][3], t_est[3], R_true[3][3], t_true[3];
            double err2 = PnP.compute_pose(R_est, t_est);

            Matrix4d RT_object2lighthouse_est, RT_object2lighthouse_est_backup;

            RT_object2lighthouse_est.setIdentity();
            RT_object2lighthouse_est.block(0, 0, 3, 3) = Map<Matrix3d>(&R_est[0][0], 3, 3);
            RT_object2lighthouse_est.block(0, 3, 3, 1) = Map<Vector3d>(&t_est[0], 3, 1);
            RT_object2lighthouse_est_backup = RT_object2lighthouse_est;
            RT_object2lighthouse_est.block(0, 0, 3, 1) = -1.0 * RT_object2lighthouse_est_backup.block(0, 1, 3, 1);
            RT_object2lighthouse_est.block(0, 2, 3, 1) = RT_object2lighthouse_est_backup.block(0, 0, 3, 1);
            RT_object2lighthouse_est.block(0, 1, 3, 1) = -1.0 * RT_object2lighthouse_est_backup.block(0, 2, 3, 1);
            RT_object2lighthouse_est.block(0, 3, 3, 1) << RT_object2lighthouse_est_backup(0, 3),
                    RT_object2lighthouse_est_backup(2, 3),
                    -RT_object2lighthouse_est_backup(1, 3);

            ROS_INFO_STREAM_THROTTLE(5, endl << RT_object2lighthouse_est.format(fmt) << endl
                                             << RT_object2lighthouse_true.format(fmt));

            tf::Transform tf;
            getTFtransform(RT_object2lighthouse_est, tf);
            string tf_name = name + "_epnp_2";
            publishTF(tf, "lighthouse2", tf_name.c_str());

            // this is the true pose (only available when simulating of course)
            if (getTransform(name.c_str(), "lighthouse2", RT_object2lighthouse_true)) {
                double rot_err, transl_err;
                Map<Matrix3d>(&R_true[0][0], 3, 3) = RT_object2lighthouse_true.block(0, 0, 3, 3);
                Map<Vector3d>(&t_true[0], 3, 1) = RT_object2lighthouse_true.block(0, 3, 3, 1);
                PnP.relative_error(rot_err, transl_err, R_true, t_true, R_est, t_est);
                stringstream str;
                str << "Reprojection error: " << err2 << endl;
                str << "rot_err: " << rot_err << ", transl_err: " << transl_err << endl;
                str << endl;
                str << "'True reprojection error':"
                    << PnP.reprojection_error(R_true, t_true) << endl;
                ROS_INFO_STREAM_THROTTLE(5, str.str());
            }
        } else {
            ROS_ERROR_THROTTLE(5, "we need at least four sensors for to be visible for lighthouse 2 for this to work, "
                    "but there are only %ld sensors visible aborting", visible_sensors[LIGHTHOUSE_B].size());
        }

        rate.sleep();
    }
}

void LighthouseEstimator::estimateObjectPoseMultiLighthouse() {
    ros::Rate rate(60);
    Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
    while (poseestimating_multiLighthouse) {
        vector<int> visible_sensors[2];
        getVisibleCalibratedSensors(LIGHTHOUSE_A, visible_sensors[LIGHTHOUSE_A]);
        getVisibleCalibratedSensors(LIGHTHOUSE_B, visible_sensors[LIGHTHOUSE_B]);

        if (visible_sensors[LIGHTHOUSE_A].size() + visible_sensors[LIGHTHOUSE_B].size() < 6) {
            ROS_ERROR_THROTTLE(1, "we need at least six visible sensors for this to work, "
                    "but there are only %ld sensors visible aborting",
                               visible_sensors[LIGHTHOUSE_A].size() + visible_sensors[LIGHTHOUSE_B].size());
            continue;
        }

        VectorXd pose(6);

        vector<Vector4d> rel_positions;
        vector<double> elevations, azimuths;
        vector<int> lighthouse_id;
        for (auto sensor:visible_sensors[LIGHTHOUSE_A]) {
            Vector4d rel_pos;
            sensors[sensor].get(LIGHTHOUSE_A, elevations, azimuths);
            sensors[sensor].getRelativeLocation(rel_pos);
            rel_positions.push_back(rel_pos);
            lighthouse_id.push_back(LIGHTHOUSE_A);
        }
        for (auto sensor:visible_sensors[LIGHTHOUSE_B]) {
            Vector4d rel_pos;
            sensors[sensor].get(LIGHTHOUSE_B, elevations, azimuths);
            sensors[sensor].getRelativeLocation(rel_pos);
            rel_positions.push_back(rel_pos);
            lighthouse_id.push_back(LIGHTHOUSE_B);
        }

        PoseEstimatorMultiLighthouse::PoseEstimator estimator(
                visible_sensors[LIGHTHOUSE_A].size() + visible_sensors[LIGHTHOUSE_B].size());
        Matrix4d lighthousePose;
        getTransform("world", LIGHTHOUSE_A, lighthousePose);
        estimator.lighthousePose.push_back(lighthousePose);
        getTransform("world", LIGHTHOUSE_B, lighthousePose);
        estimator.lighthousePose.push_back(lighthousePose);
        estimator.rel_pos = rel_positions;
        estimator.azimuths = azimuths;
        estimator.elevations = elevations;
        estimator.lighthouse_id = lighthouse_id;

        ROS_INFO_STREAM_THROTTLE(1, estimator.lighthousePose[LIGHTHOUSE_A].format(fmt) << endl
                                                                                       << estimator.lighthousePose[LIGHTHOUSE_B].format(
                                                                                               fmt));

        pose << 0, 0, 0, 0, 0, 0.0001;

        NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator> *numDiff;
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator>, double> *lm;
        numDiff = new NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator>(estimator);
        lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator>, double>(*numDiff);
        lm->parameters.maxfev = MAX_ITERATIONS;
        lm->parameters.xtol = 1e-10;
        int ret = lm->minimize(pose);
        ROS_INFO_THROTTLE(1,
                          "object pose estimation using %ld sensors, finished after %ld iterations, with an error of %f",
                          visible_sensors[LIGHTHOUSE_A].size() + visible_sensors[LIGHTHOUSE_B].size(), lm->iter,
                          lm->fnorm);

        Matrix4d RT_object;
        getRTmatrix(RT_object, pose);

        tf::Transform tf;
        getTFtransform(RT_object, tf);
        publishTF(tf, "world", "object_multi_lighthouse");

        rate.sleep();
    }
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
                        ray0 *= 4.0;
                        publishRay(pos, ray0, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
                                   COLOR(0, 1, 0, 1.0), 1);
                        ray1 *= 4.0;
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
    map<int, high_resolution_clock::time_point[4]> timestamps_old;

    ros::Rate rate(60);
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

                applyCalibrationData(lighthouse0_angles, lighthouse1_angles);

                if (lighthouse_active[LIGHTHOUSE_A]) {
                    Vector3d ray;
                    rayFromLighthouseAngles(lighthouse0_angles, ray);
                    Vector3d pos(0, 0, 0);
                    ray *= 4.0;
                    publishRay(pos, ray, "lighthouse1", "rays_lighthouse_1", getMessageID(RAY, sensor.first, 0),
                               COLOR(1, 0, 0, 0.5), 1);
                }
                if (lighthouse_active[LIGHTHOUSE_B]) {
                    Vector3d ray;
                    rayFromLighthouseAngles(lighthouse1_angles, ray);
                    Vector3d pos(0, 0, 0);
                    ray *= 4.0;
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

bool LighthouseEstimator::estimateFactoryCalibration(int lighthouse) {
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
        if (active_sensors.size() < 20 && (ros::Duration(ros::Time::now() - t0).toSec() > 10)) {
            ROS_ERROR("not enough active sensors (%ld), giving up", active_sensors.size());
            return false;
        }
    }
    ROS_INFO("%ld active sensors", active_sensors.size());

    elevation_measured.clear();
    azimuth_measured.clear();
    elevation_truth.clear();
    azimuth_truth.clear();
    for (auto &active_sensor:active_sensors) {
        // calculate the ground truth
        Vector4d rel_pos;
        sensors[active_sensor].getRelativeLocation(rel_pos);
        // transform into lighthouse frame
        Vector4d pos_in_lighthouse_frame = RT_object2lighthouse * rel_pos;
        Vector4d sensor_pos_motor_vertical, sensor_pos_motor_horizontal;
        sensor_pos_motor_vertical = pos_in_lighthouse_frame - Vector4d(-AXIS_OFFSET, 0, 0, 0);
        sensor_pos_motor_horizontal = pos_in_lighthouse_frame - Vector4d(0, 0, -AXIS_OFFSET, 0);

        str << active_sensor << " sensor position in ligthhouse frame: " << pos_in_lighthouse_frame[0] << " "
            << pos_in_lighthouse_frame[1] << " " << pos_in_lighthouse_frame[2] << endl;

        // calculate ground truth ligthhouse angles
        double elevation = M_PI - atan2(sensor_pos_motor_vertical[1], sensor_pos_motor_vertical[2]);
        double azimuth = atan2(sensor_pos_motor_horizontal[1], sensor_pos_motor_horizontal[0]);
        vector<double> a = {elevation, azimuth};
        elevation_truth.push_back(elevation);
        azimuth_truth.push_back(azimuth);

        // get the measured angles
        sensors[active_sensor].get(lighthouse, elevation, azimuth);
        elevation_measured.push_back(elevation);
        azimuth_measured.push_back(azimuth);

        str << "\t true " << elevation_truth.back() << "\t" << azimuth_truth.back() << endl
            << "\t measured " << elevation_measured.back() << "\t" << azimuth_measured.back() << endl;
    }
    ROS_INFO_STREAM(str.str());
    numberOfSensors = elevation_measured.size();

    InYourGibbousPhase::InYourGibbousPhase estimator(elevation_measured.size());
    estimator.elevation_measured = elevation_measured;
    estimator.azimuth_measured = azimuth_measured;
    estimator.elevation_truth = elevation_truth;
    estimator.azimuth_truth = azimuth_truth;

    NumericalDiff<InYourGibbousPhase::InYourGibbousPhase> *numDiff1;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<InYourGibbousPhase::InYourGibbousPhase>, double> *lm;
    numDiff1 = new NumericalDiff<InYourGibbousPhase::InYourGibbousPhase>(estimator);
    lm = new LevenbergMarquardt<NumericalDiff<InYourGibbousPhase::InYourGibbousPhase>, double>(*numDiff1);
    lm->parameters.maxfev = 1000;
    lm->parameters.xtol = 1e-10;
    VectorXd x(10);
    x << 0.0000001, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    int ret = lm->minimize(x);
    double error = lm->fnorm;

//    real_1d_array x;
//    x.setlength(10);
//    for(int i=0;i<10;i++)
//        x[i] = 0;
//    double epsg = 0.0000000001, epsx = 0.0000000001, epsf = 0.0000000001;
//    ae_int_t maxits = 0;
//    minlmstate state;
//    minlmreport rep;
//
//    minlmcreatev(10, 2, x, 0.000001, state);
//    minlmsetcond(state, epsg, epsf, epsx, maxits);
//    alglib::minlmoptimize(state, inYourGibbousPhase);
//    minlmresults(state, x, rep);
//    double error = 0;
//    for(int i=0; i<elevation_measured.size(); i++){
//        error += pow(elevation_measured[i]-elevation_truth[i],2.0);
//        error += pow(azimuth_measured[i]-azimuth_truth[i],2.0);
//    }
//    error = sqrt(error)/elevation_measured.size(); // mse

    ROS_INFO_STREAM(
            "calibration value estimation for lighthouse " << lighthouse + 1 << " terminated with error " << error
                                                           << endl
                                                           << "phase horizontal    "
                                                           << x[phase_horizontal]
                                                           << endl
                                                           << "phase vertical      "
                                                           << x[phase_vertical]
                                                           << endl
                                                           << "tilt horizontal     "
                                                           << x[tilt_horizontal]
                                                           << endl
                                                           << "tilt vertical       "
                                                           << x[tilt_vertical]
                                                           << endl
                                                           << "curve horizontal    "
                                                           << x[curve_horizontal]
                                                           << endl
                                                           << "curve vertical      "
                                                           << x[curve_vertical]
                                                           << endl
                                                           << "gibphase horizontal "
                                                           << x[gibphase_horizontal]
                                                           << endl
                                                           << "gibphase vertical   "
                                                           << x[gibphase_vertical]
                                                           << endl
                                                           << "gibmag horizontal   "
                                                           << x[gibmag_horizontal]
                                                           << endl
                                                           << "gibmag vertical     "
                                                           << x[gibmag_vertical]
                                                           << endl);

    calibration[lighthouse][VERTICAL].phase = x[phase_vertical];
    calibration[lighthouse][HORIZONTAL].phase = x[phase_horizontal];
    calibration[lighthouse][VERTICAL].tilt = x[tilt_vertical];
    calibration[lighthouse][HORIZONTAL].tilt = x[tilt_horizontal];
    calibration[lighthouse][VERTICAL].curve = x[curve_vertical];
    calibration[lighthouse][HORIZONTAL].curve = x[curve_horizontal];
    calibration[lighthouse][VERTICAL].gibphase = x[gibphase_vertical];
    calibration[lighthouse][HORIZONTAL].gibphase = x[gibphase_horizontal];
    calibration[lighthouse][VERTICAL].gibmag = x[gibmag_vertical];
    calibration[lighthouse][HORIZONTAL].gibmag = x[gibmag_horizontal];

    string package_path = ros::package::getPath("darkroom");
    string calibration_result_path = package_path + "/params/lighthouse_calibration.yaml";

    return writeCalibrationConfig(calibration_result_path, lighthouse, calibration[lighthouse]);
}

bool LighthouseEstimator::estimateFactoryCalibrationEPNP(int lighthouse) {
    ros::Rate rate(30);
    Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
    ros::Time t0 = ros::Time::now();
    vector<double> elevations_measured, azimuths_measured, elevations_model, azimuths_model;
    double error;
    do {
        vector<int> visible_sensors;
        getVisibleCalibratedSensors(lighthouse, visible_sensors);
        Matrix4d RT_object2lighthouse_true;

        if (visible_sensors.size() >= 4) {
            epnp PnP;
            PnP.set_internal_parameters(0, 0, 1, 1);
            PnP.set_maximum_number_of_correspondences(visible_sensors.size());
            PnP.reset_correspondences();
            for (int sensor:visible_sensors) {
                Vector3d rel_pos;
                sensors[sensor].getRelativeLocation(rel_pos);
                Vector2d angles;
                sensors[sensor].get(lighthouse, angles);
                elevations_measured.push_back(angles[VERTICAL]);
                azimuths_measured.push_back(angles[HORIZONTAL]);
                applyCalibrationData(lighthouse, angles);
                PnP.add_correspondence(rel_pos[2], rel_pos[1], rel_pos[0], tan(M_PI_2 - angles[HORIZONTAL]),
                                       tan(M_PI_2 - angles[VERTICAL]));
            }

            double R_est[3][3], t_est[3], R_true[3][3], t_true[3];
            double err2 = PnP.compute_pose(R_est, t_est);

            Matrix4d RT_object2lighthouse_est, RT_object2lighthouse_est_backup;

            RT_object2lighthouse_est.setIdentity();
            RT_object2lighthouse_est.block(0, 0, 3, 3) = Map<Matrix3d>(&R_est[0][0], 3, 3);
            RT_object2lighthouse_est.block(0, 3, 3, 1) = Map<Vector3d>(&t_est[0], 3, 1);
            RT_object2lighthouse_est_backup = RT_object2lighthouse_est;
            RT_object2lighthouse_est.block(0, 0, 3, 1) = -1.0 * RT_object2lighthouse_est_backup.block(0, 1, 3, 1);
            RT_object2lighthouse_est.block(0, 2, 3, 1) = RT_object2lighthouse_est_backup.block(0, 0, 3, 1);
            RT_object2lighthouse_est.block(0, 1, 3, 1) = -1.0 * RT_object2lighthouse_est_backup.block(0, 2, 3, 1);
            RT_object2lighthouse_est.block(0, 3, 3, 1) << RT_object2lighthouse_est_backup(0, 3),
                    RT_object2lighthouse_est_backup(2, 3),
                    -RT_object2lighthouse_est_backup(1, 3);

            ROS_INFO_STREAM_THROTTLE(5, RT_object2lighthouse_est.format(fmt) << endl
                                                                             << RT_object2lighthouse_true.format(fmt));

            tf::Transform tf;
            getTFtransform(RT_object2lighthouse_est, tf);
            string tf_name = name + "_" + (lighthouse ? "epnp2" : "epnp1");
            publishTF(tf, (lighthouse ? "lighthouse2" : "lighthouse1"), tf_name.c_str());

            // this is the true pose (only available when simulating of course)
            if (getTransform(name.c_str(), (lighthouse == LIGHTHOUSE_A ? "lighthouse1" : "lighthouse2"),
                             RT_object2lighthouse_true)) {
                double rot_err, transl_err;
                Map<Matrix3d>(&R_true[0][0], 3, 3) = RT_object2lighthouse_true.block(0, 0, 3, 3);
                Map<Vector3d>(&t_true[0], 3, 1) = RT_object2lighthouse_true.block(0, 3, 3, 1);
                PnP.relative_error(rot_err, transl_err, R_true, t_true, R_est, t_est);
                stringstream str;
                str << ">>> Reprojection error: " << err2 << endl;
                str << ">>> rot_err: " << rot_err << ", transl_err: " << transl_err << endl;
                str << endl;
                str << "'True reprojection error':"
                    << PnP.reprojection_error(R_true, t_true) << endl;
                ROS_INFO_STREAM_THROTTLE(5, str.str());
            }

            // using the pose from epnp we calculate the new lighthouse angles
            for (int sensor:visible_sensors) {
                Vector4d sensor_pos, rel_pos;
                sensors[sensor].getRelativeLocation(rel_pos);
                sensor_pos = RT_object2lighthouse_est * rel_pos;

                Vector4d sensor_pos_motor_vertical, sensor_pos_motor_horizontal;
                sensor_pos_motor_vertical = sensor_pos - Vector4d(-AXIS_OFFSET, 0, 0, 0);
                sensor_pos_motor_horizontal = sensor_pos - Vector4d(0, 0, -AXIS_OFFSET, 0);

                double elevation = M_PI - atan2(sensor_pos_motor_vertical(1), sensor_pos_motor_vertical(2));
                double azimuth = atan2(sensor_pos_motor_horizontal[1], sensor_pos_motor_horizontal[0]);

                ROS_DEBUG_STREAM_THROTTLE(1, "measured sensor pos: " << sensor_pos.transpose() << "\t elevation "
                                                                     << elevation << "\t azimuth " << azimuth);

//                // apply our calibration model to get the lighthouse angles we expect
//                double temp_elevation1 = calibration[lighthouse][VERTICAL].curve*pow(cos(azimuth)*sin(elevation),2.0);
//                double temp_elevation2 = calibration[lighthouse][VERTICAL].gibmag*cos(elevation+calibration[lighthouse][VERTICAL].gibphase);
//                double temp_azimuth1 = calibration[lighthouse][HORIZONTAL].curve*pow(-sin(azimuth)*cos(elevation),2.0);
//                double temp_azimuth2 = calibration[lighthouse][HORIZONTAL].gibmag*cos(azimuth+calibration[lighthouse][HORIZONTAL].gibphase);
//                elevation -= (calibration[lighthouse][VERTICAL].phase + temp_elevation1 + temp_elevation2);
//                azimuth -= (calibration[lighthouse][HORIZONTAL].phase + temp_azimuth1 + temp_azimuth2);

                elevations_model.push_back(elevation);
                azimuths_model.push_back(azimuth);
            }

//            InYourGibbousPhase3::InYourGibbousPhase3 estimator(elevations_model.size());
//            estimator.elevation_measured = elevations_measured;
//            estimator.azimuth_measured = azimuths_measured ;
//            estimator.elevation_model = elevations_model ;
//            estimator.azimuth_model = azimuths_model;
//
//            NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3> *numDiff1;
//            Eigen::LevenbergMarquardt<Eigen::NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3>, double> *lm;
//            numDiff1 = new NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3>(estimator);
//            lm = new LevenbergMarquardt<NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3>, double>(*numDiff1);
//            lm->parameters.maxfev = 1000;
//            lm->parameters.xtol = 1e-10;
//            VectorXd x(10);
//            x << calibration[lighthouse][HORIZONTAL].phase,
//                    calibration[lighthouse][VERTICAL].phase,
//                    calibration[lighthouse][HORIZONTAL].tilt,
//                    calibration[lighthouse][VERTICAL].tilt,
//                    calibration[lighthouse][HORIZONTAL].curve,
//                    calibration[lighthouse][VERTICAL].curve,
//                    calibration[lighthouse][HORIZONTAL].gibphase,
//                    calibration[lighthouse][VERTICAL].gibphase,
//                    calibration[lighthouse][HORIZONTAL].gibmag,
//                    calibration[lighthouse][VERTICAL].gibmag;
//            int ret = lm->minimize(x);
//            error = lm->fnorm;
//
//            ROS_INFO_STREAM(
//                    "calibration value estimation for lighthouse " << lighthouse + 1 << " terminated with error " << error
//                                                                   << endl
//                                                                   << "phase horizontal    "
//                                                                   << x[phase_horizontal]
//                                                                   << endl
//                                                                   << "phase vertical      "
//                                                                   << x[phase_vertical]
//                                                                   << endl
//                                                                   << "tilt horizontal     "
//                                                                   << x[tilt_horizontal]
//                                                                   << endl
//                                                                   << "tilt vertical       "
//                                                                   << x[tilt_vertical]
//                                                                   << endl
//                                                                   << "curve horizontal    "
//                                                                   << x[curve_horizontal]
//                                                                   << endl
//                                                                   << "curve vertical      "
//                                                                   << x[curve_vertical]
//                                                                   << endl
//                                                                   << "gibphase horizontal "
//                                                                   << x[gibphase_horizontal]
//                                                                   << endl
//                                                                   << "gibphase vertical   "
//                                                                   << x[gibphase_vertical]
//                                                                   << endl
//                                                                   << "gibmag horizontal   "
//                                                                   << x[gibmag_horizontal]
//                                                                   << endl
//                                                                   << "gibmag vertical     "
//                                                                   << x[gibmag_vertical]);
//
//            calibration[lighthouse][HORIZONTAL].phase = x[phase_horizontal];
//            calibration[lighthouse][VERTICAL].phase = x[phase_vertical];
//            calibration[lighthouse][HORIZONTAL].tilt = x[tilt_horizontal];
//            calibration[lighthouse][VERTICAL].tilt = x[tilt_vertical];
//            calibration[lighthouse][HORIZONTAL].curve = x[curve_horizontal];
//            calibration[lighthouse][VERTICAL].curve = x[curve_vertical];
//            calibration[lighthouse][HORIZONTAL].gibphase = x[gibphase_horizontal];
//            calibration[lighthouse][VERTICAL].gibphase = x[gibphase_vertical];
//            calibration[lighthouse][HORIZONTAL].gibmag = x[gibmag_horizontal];
//            calibration[lighthouse][VERTICAL].gibmag = x[gibmag_vertical];

        } else {
            ROS_ERROR_THROTTLE(5, "we need at least four sensors to be visible for this to work, "
                    "but there are only %ld sensors visible aborting", visible_sensors.size());
            error = 1;
        }

        rate.sleep();
    } while (ros::Duration(ros::Time::now() - t0).toSec() < 100 && error > 0.00001);

    ROS_INFO("calibration terminated with error %lf", error);
}

bool LighthouseEstimator::estimateFactoryCalibrationMultiLighthouse(int lighthouse) {
    ros::Rate rate(1);
    Eigen::IOFormat fmt(4, 0, " ", ";\n", "", "", "[", "]");
    vector<vector<double>> elevations_measured, azimuths_measured;
    vector<Matrix4d> object_pose_truth;
    Matrix4d lighthousePose[2], world2lighthouse;

    while (!getTransform("world", lighthouse, world2lighthouse))
        ROS_INFO_THROTTLE(1, "waiting for transform to lighthouse");
    while (!getTransform("world", LIGHTHOUSE_A, lighthousePose[LIGHTHOUSE_A]))
        ROS_INFO_THROTTLE(1, "waiting for transform to lighthouse 1");
    while (!getTransform("world", LIGHTHOUSE_B, lighthousePose[LIGHTHOUSE_B]))
        ROS_INFO_THROTTLE(1, "waiting for transform to lighthouse 2");

    ROS_INFO_STREAM_THROTTLE(1, "lighthouse poses: " << endl << lighthousePose[LIGHTHOUSE_A].format(fmt) << endl
                                                     << lighthousePose[LIGHTHOUSE_B].format(fmt) << endl <<
                                                     "world to lighthouse: " << endl << world2lighthouse);
    vector<vector<Vector4d>> rel_positions;
    vector<vector<int>> lighthouse_ids;

    int measurement_time = 5;
    if (nh->hasParam("measurement_time"))
        nh->getParam("measurement_time", measurement_time);

    ROS_INFO("recording lighthouse angles for %d seconds", measurement_time);
    ros::Time t0 = ros::Time::now();
    int iter = 0;
    do {
        Matrix4d object_pose;
        getTransform(name.c_str(), "world", object_pose);
        object_pose_truth.push_back(object_pose);

        Vector3d pos = object_pose.block(0, 3, 3, 1);
        publishSphere(pos, "world", "recorded trajectory", (iter++) + 66666, COLOR(0, 1, 0, 0.2));

        vector<int> visible_sensors;
        getVisibleCalibratedSensors(lighthouse, visible_sensors);

        if (visible_sensors.size() < 6) {
            ROS_ERROR_THROTTLE(1, "we need at least six visible sensors for this to work, "
                    "but there are only %ld sensors visible aborting",
                               visible_sensors.size());
            continue;
        }

        vector<Vector4d> rel_pos;
        vector<int> lighthouse_id;
        vector<double> elevations, azimuths;
        for (auto sensor:visible_sensors) {
            Vector4d rp;
            sensors[sensor].get(lighthouse, elevations, azimuths);
            sensors[sensor].getRelativeLocation(rp);
            rel_pos.push_back(rp);
            lighthouse_id.push_back(lighthouse);
        }


        elevations_measured.push_back(elevations);
        azimuths_measured.push_back(azimuths);
        rel_positions.push_back(rel_pos);
        lighthouse_ids.push_back(lighthouse_id);

        rate.sleep();
    } while (ros::Duration(ros::Time::now() - t0).toSec() < measurement_time);

    ROS_INFO("recorded %ld frames, starting estimation of calibration values", azimuths_measured.size());
    double error;

    int iteration = 0, max_iterations = 10;

    do {
////        vector<vector<double>> elevations_calib, azimuths_calib;
////        elevations_calib = elevations_measured;
////        azimuths_calib = azimuths_measured;
//        error = 0;
//        for (int frame = 0; frame < elevations_measured.size(); frame++) {
////            for(int i=0;i<elevations_measured[frame].size();i++){
////                applyCalibrationData(lighthouse,elevations_calib[frame][i],azimuths_calib[frame][i]);
////            }
//
//            InYourGibbousPhase4::PoseEstimator estimator(elevations_measured[frame].size());
//            estimator.lighthousePose.push_back(lighthousePose[LIGHTHOUSE_A]);
//            estimator.lighthousePose.push_back(lighthousePose[LIGHTHOUSE_B]);
//            estimator.rel_pos = rel_positions[frame];
//            estimator.elevations = elevations_measured[frame];
//            estimator.azimuths = azimuths_measured[frame];
//            estimator.lighthouse_id = lighthouse_ids[frame];
//
//            VectorXd x(12);
//            x << 0, 0, 0, 0, 0, 0.0001, calibration[lighthouse][HORIZONTAL].curve, calibration[lighthouse][VERTICAL].curve,
//                    calibration[lighthouse][HORIZONTAL].gibphase, calibration[lighthouse][VERTICAL].gibphase,
//                    calibration[lighthouse][HORIZONTAL].gibmag, calibration[lighthouse][VERTICAL].gibmag;
//
//            NumericalDiff<InYourGibbousPhase4::PoseEstimator> *numDiff;
//            Eigen::LevenbergMarquardt<Eigen::NumericalDiff<InYourGibbousPhase4::PoseEstimator>, double> *lm;
//            numDiff = new NumericalDiff<InYourGibbousPhase4::PoseEstimator>(estimator);
//            lm = new LevenbergMarquardt<NumericalDiff<InYourGibbousPhase4::PoseEstimator>, double>(*numDiff);
//            lm->parameters.maxfev = MAX_ITERATIONS;
//            lm->parameters.xtol = 1e-10;
//            int ret = lm->minimize(x);
//            ROS_INFO_THROTTLE(1,
//                              "object pose estimation using %ld sensors, finished after %ld iterations, with an error of %f",
//                              elevations_measured[frame].size(), lm->iter, lm->fnorm);
//            error += lm->fnorm;
//
//            VectorXd pose(6);
//            pose << x(0),x(1),x(2),x(3),x(4),x(5);
//            Matrix4d RT_object;
//            getRTmatrix(RT_object, pose);
//
//            Vector3d pos = RT_object.block(0,3,3,1);
//            publishSphere(pos,"world","estimated trajectory",frame+77777,COLOR(1,0,0,0.2));
//
//            tf::Transform tf;
//            getTFtransform(RT_object, tf);
//            publishTF(tf, "world", "object_multi_lighthouse");
//            getTFtransform(object_pose_truth[frame], tf);
//            publishTF(tf, "world", "object_multi_lighthouse_truth");
//
//            calibration[lighthouse][HORIZONTAL].curve = 0.9*calibration[lighthouse][HORIZONTAL].curve + 0.1*x[6];
//            calibration[lighthouse][VERTICAL].curve = 0.9*calibration[lighthouse][VERTICAL].curve + 0.1*x[7];
//            calibration[lighthouse][HORIZONTAL].gibphase = 0.9*calibration[lighthouse][HORIZONTAL].gibphase + 0.1*x[8];
//            calibration[lighthouse][VERTICAL].gibphase = 0.9*calibration[lighthouse][VERTICAL].gibphase + 0.1*x[9];
//            calibration[lighthouse][HORIZONTAL].gibmag = 0.9*calibration[lighthouse][HORIZONTAL].gibmag + 0.1*x[10];
//            calibration[lighthouse][VERTICAL].gibmag = 0.9*calibration[lighthouse][VERTICAL].gibmag + 0.1*x[11];
//        }


        vector<vector<double>> elevations_model, azimuths_model, elevations_calib, azimuths_calib;
        elevations_calib = elevations_measured;
        azimuths_calib = azimuths_measured;
        for (int frame = 0; frame < elevations_measured.size(); frame++) {
            for (int i = 0; i < elevations_measured[frame].size(); i++) {
                applyCalibrationData(lighthouse, elevations_calib[frame][i], azimuths_calib[frame][i]);
            }

            PoseEstimatorMultiLighthouse::PoseEstimator estimator(elevations_measured[frame].size());
            estimator.lighthousePose.push_back(lighthousePose[LIGHTHOUSE_A]);
            estimator.lighthousePose.push_back(lighthousePose[LIGHTHOUSE_B]);
            estimator.rel_pos = rel_positions[frame];
            estimator.elevations = elevations_calib[frame];
            estimator.azimuths = azimuths_calib[frame];
            estimator.lighthouse_id = lighthouse_ids[frame];

            VectorXd pose(6);
            pose << 0, 0, 0, 0, 0, 0.0001;

            NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator> *numDiff;
            Eigen::LevenbergMarquardt<Eigen::NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator>, double> *lm;
            numDiff = new NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator>(estimator);
            lm = new LevenbergMarquardt<NumericalDiff<PoseEstimatorMultiLighthouse::PoseEstimator>, double>(*numDiff);
            lm->parameters.maxfev = MAX_ITERATIONS;
            lm->parameters.xtol = 1e-10;
            int ret = lm->minimize(pose);
//            ROS_INFO_THROTTLE(1,
//                              "object pose estimation using %ld sensors, finished after %ld iterations, with an error of %f",
//                              elevations_measured[frame].size(), lm->iter, lm->fnorm);

            Matrix4d RT_object;
            getRTmatrix(RT_object, pose);

            Vector3d pos = RT_object.block(0, 3, 3, 1);
            publishSphere(pos, "world", "estimated trajectory", frame + 77777, COLOR(1, 0, 0, 0.2));

            tf::Transform tf;
            getTFtransform(RT_object, tf);
            publishTF(tf, "world", "object_multi_lighthouse");
            getTFtransform(object_pose_truth[frame], tf);
            publishTF(tf, "world", "object_multi_lighthouse_truth");

            vector<double> elevation_model, azimuth_model;
            // using the pose we calculate the new lighthouse angles
            int ray_counter = 100000;
            for (Vector4d &rel_pos:rel_positions[frame]) {
                Vector4d sensor_pos;
                sensor_pos = world2lighthouse * RT_object * rel_pos;

                Vector4d sensor_pos_motor_vertical, sensor_pos_motor_horizontal;
                sensor_pos_motor_vertical = sensor_pos - Vector4d(-AXIS_OFFSET, 0, 0, 0);
                sensor_pos_motor_horizontal = sensor_pos - Vector4d(0, 0, -AXIS_OFFSET, 0);

                double elevation = M_PI - atan2(sensor_pos_motor_vertical(1), sensor_pos_motor_vertical(2));
                double azimuth = atan2(sensor_pos_motor_horizontal[1], sensor_pos_motor_horizontal[0]);

//                Vector3d ray,pos(0,0,0);
//                rayFromLighthouseAngles(elevation,azimuth,ray);
//                publishRay(pos,ray,(lighthouse?"ligthhouse2":"lighthouse1"),"rays",ray_counter++,COLOR(0,0,1,1));

//                ROS_INFO_STREAM_THROTTLE(1, "measured sensor pos: " << sensor_pos.transpose() << "\t elevation "
//                                                                     << elevation << "\t azimuth " << azimuth);

//                applyCalibrationData(lighthouse,elevation,azimuth);

                elevation_model.push_back(elevation);
                azimuth_model.push_back(azimuth);
            }
            elevations_model.push_back(elevation_model);
            azimuths_model.push_back(azimuth_model);
//            ros::Duration d(1);
//            d.sleep();
        }


        InYourGibbousPhase3::InYourGibbousPhase3 estimator2(elevations_model.size());
        estimator2.elevation_measured = elevations_measured;
        estimator2.azimuth_measured = azimuths_measured;
        estimator2.elevation_model = elevations_model;
        estimator2.azimuth_model = azimuths_model;

        NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3> *numDiff1;
        Eigen::LevenbergMarquardt<Eigen::NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3>, double> *lm2;
        numDiff1 = new NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3>(estimator2);
        lm2 = new LevenbergMarquardt<NumericalDiff<InYourGibbousPhase3::InYourGibbousPhase3>, double>(*numDiff1);
        lm2->parameters.maxfev = 1000;
        lm2->parameters.xtol = 1e-10;
        VectorXd x(10);
        x << calibration[lighthouse][HORIZONTAL].phase,
                calibration[lighthouse][VERTICAL].phase,
                calibration[lighthouse][HORIZONTAL].tilt,
                calibration[lighthouse][VERTICAL].tilt,
                calibration[lighthouse][HORIZONTAL].curve,
                calibration[lighthouse][VERTICAL].curve,
                calibration[lighthouse][HORIZONTAL].gibphase,
                calibration[lighthouse][VERTICAL].gibphase,
                calibration[lighthouse][HORIZONTAL].gibmag,
                calibration[lighthouse][VERTICAL].gibmag;
        int ret2 = lm2->minimize(x);
        error = lm2->fnorm;

        ROS_INFO_STREAM(
                "calibration value estimation for lighthouse " << lighthouse + 1 << " terminated with error " << error
                                                               << endl
                                                               << "phase horizontal    "
                                                               << x[phase_horizontal]
                                                               << endl
                                                               << "phase vertical      "
                                                               << x[phase_vertical]
                                                               << endl
                                                               << "tilt horizontal     "
                                                               << x[tilt_horizontal]
                                                               << endl
                                                               << "tilt vertical       "
                                                               << x[tilt_vertical]
                                                               << endl
                                                               << "curve horizontal    "
                                                               << x[curve_horizontal]
                                                               << endl
                                                               << "curve vertical      "
                                                               << x[curve_vertical]
                                                               << endl
                                                               << "gibphase horizontal "
                                                               << x[gibphase_horizontal]
                                                               << endl
                                                               << "gibphase vertical   "
                                                               << x[gibphase_vertical]
                                                               << endl
                                                               << "gibmag horizontal   "
                                                               << x[gibmag_horizontal]
                                                               << endl
                                                               << "gibmag vertical     "
                                                               << x[gibmag_vertical]);

        calibration[lighthouse][HORIZONTAL].phase = x[phase_horizontal];
        calibration[lighthouse][VERTICAL].phase = x[phase_vertical];
        calibration[lighthouse][HORIZONTAL].tilt = x[tilt_horizontal];
        calibration[lighthouse][VERTICAL].tilt = x[tilt_vertical];
        calibration[lighthouse][HORIZONTAL].curve = x[curve_horizontal];
        calibration[lighthouse][VERTICAL].curve = x[curve_vertical];
        calibration[lighthouse][HORIZONTAL].gibphase = x[gibphase_horizontal];
        calibration[lighthouse][VERTICAL].gibphase = x[gibphase_vertical];
        calibration[lighthouse][HORIZONTAL].gibmag = x[gibmag_horizontal];
        calibration[lighthouse][VERTICAL].gibmag = x[gibmag_vertical];

        iteration++;
        if (nh->hasParam("max_iterations"))
            nh->getParam("max_iterations", max_iterations);
    } while (error > 0.00001 && iteration < max_iterations);
    ROS_INFO("calibration terminated with error %lf", error);
}

bool LighthouseEstimator::estimateFactoryCalibration2(int lighthouse) {
    stringstream str;
    // get the transform from object to lighthouse
    Matrix4d RT_object2lighthouse;
    ros::Time t0 = ros::Time::now();
    ros::Rate rate(1);
    int trajectory_points = 0;
    relative_positions_trajectory.clear();
    angles_measured_trajectory.clear();
    vector<Vector3d> absolute_position;
    double measurement_time = 5;
    if (nh->hasParam("measurement_time"))
        nh->getParam("measurement_time", measurement_time);
    while ((ros::Duration(ros::Time::now() - t0).toSec() < measurement_time)) {
        // lets see who is active
        vector<int> active_sensors;
        vector<Vector2d> angles_measured;
        vector<Vector3d> relPositions;
        getVisibleCalibratedSensors(lighthouse, active_sensors);
        Matrix4d RT_object2world;
        getTransform(name.c_str(), "world", RT_object2world);
        for (auto &active_sensor:active_sensors) {
            double elevation, azimuth;
            // get the measured angles
            Vector2d angles;
            sensors[active_sensor].get(lighthouse, angles);
            angles_measured.push_back(angles);
            Vector4d rel_pos;
            sensors[active_sensor].getRelativeLocation(rel_pos);
            relPositions.push_back(Vector3d(rel_pos(0), rel_pos(1), rel_pos(2)));
        }
        if (!active_sensors.empty()) {
            relative_positions_trajectory.push_back(relPositions);
            angles_measured_trajectory.push_back(angles_measured);
            absolute_position.push_back(RT_object2world.block(0, 3, 3, 1));
            trajectory_points++;

            rate.sleep();
        }
    }

    stringstream s;
    for (int i = 0; i < trajectory_points; i++) {
        s << "trajectory point " << i << endl;
        for (int j = 0; j < relative_positions_trajectory[i].size(); j++) {
            s << angles_measured_trajectory[i][j].transpose() << "\t\t"
              << relative_positions_trajectory[i][j].transpose() << endl;
        }
    }

    ROS_INFO_STREAM(s.str());

    real_1d_array x;
    x.setlength(trajectory_points * 2);
    double epsg = 0.0000000001, epsx = 0.0000000001, epsf = 0.0000000001;
    ae_int_t maxits = 0;
    minlmstate state;
    minlmreport rep;

    minlmcreatev(trajectory_points * 2, x, 0.0001, state);
    minlmsetcond(state, epsg, epsf, epsx, maxits);
    alglib::minlmoptimize(state, function1_fvec);
    minlmresults(state, x, rep);


//    real_1d_array x;
//    x.setlength(10+trajectory_points);
//    real_1d_array bndl;
//    bndl.setlength(10+trajectory_points);
//    for(int i=0;i<10+trajectory_points;i++)
//        bndl[i] = -1;
//    real_1d_array bndu;
//    bndu.setlength(10+trajectory_points);
//    for(int i=0;i<10+trajectory_points;i++)
//        bndu[i] = 1;
//    minbleicstate state;
//    minbleicreport rep;
//    double epsg = 0.000001;
//    double epsf = 0;
//    double epsx = 0;
//    ae_int_t maxits = 0;
//    double diffstep = 1.0e-6;
//    minbleiccreatef(x, diffstep, state);
//    minbleicsetbc(state, bndl, bndu);
//    minbleicsetcond(state, epsg, epsf, epsx, maxits);
//    alglib::minbleicoptimize(state, function1_fvec);
//    minbleicresults(state, x, rep);

    switch (rep.terminationtype) {
        case -7 :
            ROS_WARN("derivative correctness check failed");
            break;
        case -3 :
            ROS_WARN("constraints are inconsistent");
            break;
        case 1:
            ROS_WARN("relative function improvement is no more than EpsF.");
            break;
        case 2:
            ROS_WARN("relative step is no more than EpsX.");
            break;
        case 4:
            ROS_WARN("gradient is no more than EpsG.");
            break;
        case 5:
            ROS_WARN("MaxIts steps was taken ");
            break;
        case 7:
            ROS_WARN("stopping conditions are too stringent, further improvement is impossible");
            break;
        case 8:
            ROS_WARN("terminated by user who called minlmrequesttermination().");
            break;
    }

    s.clear();
    double error = 0;
    Eigen::IOFormat fmt(4, 0, " ", "", "", "", "[", "]");
//    VectorXd calib(10);
//    calib << x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9];
//    s << "estimated calibration values " << calib.format(fmt) << endl;
    for (int i = 0; i < trajectory_points; i++) {
        Vector3d pos(x[(i * 2)], 0, x[(i * 2) + 1]);
        publishSphere(pos, "world", "estimated_trajectory", i + 9133534, COLOR(1.0, 0.5, 0.5, 1));
        publishSphere(absolute_position[i], "world", "true_trajectory", i + 92933239, COLOR(0, 0.5, 0.5, 1));
        s << "estimate: " << pos.format(fmt) << "  true: " << absolute_position[i].format(fmt) << endl;
        error += (absolute_position[i] - pos).norm();
    }
    ROS_INFO_STREAM(s.str());
    ROS_INFO("done with error %lf after %d iterations", error, (int) rep.iterationscount);
//
//    ROS_INFO_STREAM("calibration value estimation for lighthouse " << lighthouse + 1 << " terminated with error "
//                                                                   << lm->fnorm << " after " << lm->iter << " iterations "
//                                                                   << endl
//                                                                   << "phase horizontal    "
//                                                                   << calibrationValues(phase_horizontal)
//                                                                   << endl
//                                                                   << "phase vertical      "
//                                                                   << calibrationValues(phase_vertical)
//                                                                   << endl
//                                                                   << "tilt horizontal     "
//                                                                   << calibrationValues(tilt_horizontal)
//                                                                   << endl
//                                                                   << "tilt vertical       "
//                                                                   << calibrationValues(tilt_vertical)
//                                                                   << endl
//                                                                   << "curve horizontal    "
//                                                                   << calibrationValues(curve_horizontal)
//                                                                   << endl
//                                                                   << "curve vertical      "
//                                                                   << calibrationValues(curve_vertical)
//                                                                   << endl
//                                                                   << "gibphase horizontal "
//                                                                   << calibrationValues(gibphase_horizontal)
//                                                                   << endl
//                                                                   << "gibphase vertical   "
//                                                                   << calibrationValues(gibphase_vertical)
//                                                                   << endl
//                                                                   << "gibmag horizontal   "
//                                                                   << calibrationValues(gibmag_horizontal)
//                                                                   << endl
//                                                                   << "gibmag vertical     "
//                                                                   << calibrationValues(gibmag_vertical)
//                                                                   << endl
//                            << "trajectory     " << endl
//                            << calibrationValues.transpose()
//                            << endl
//    );
//
//    for(int i=0;i<trajectory_points;i++){
//        Vector3d pos(calibrationValues(10+(i*2)),0,calibrationValues(10+(i*2)+1));
//        publishSphere(pos,"world","estimated_trajectory", i+92933239, COLOR(0,0.5,0.5,1));
//    }
//
//    calibration[lighthouse][VERTICAL].phase = calibrationValues(phase_vertical);
//    calibration[lighthouse][HORIZONTAL].phase = calibrationValues(phase_horizontal);
//    calibration[lighthouse][VERTICAL].tilt = calibrationValues(tilt_vertical);
//    calibration[lighthouse][HORIZONTAL].tilt = calibrationValues(tilt_horizontal);
//    calibration[lighthouse][VERTICAL].curve = calibrationValues(curve_vertical);
//    calibration[lighthouse][HORIZONTAL].curve = calibrationValues(curve_horizontal);
//    calibration[lighthouse][VERTICAL].gibphase = calibrationValues(gibphase_vertical);
//    calibration[lighthouse][HORIZONTAL].gibphase = calibrationValues(gibphase_horizontal);
//    calibration[lighthouse][VERTICAL].gibmag = calibrationValues(gibmag_vertical);
//    calibration[lighthouse][HORIZONTAL].gibmag = calibrationValues(gibmag_horizontal);
//
//    string package_path = ros::package::getPath("darkroom");
//    string calibration_result_path = package_path + "/params/lighthouse_calibration.yaml";
//
//    return writeCalibrationConfig(calibration_result_path, lighthouse, calibration[lighthouse]);
}

int LighthouseEstimator::getMessageID(int type, int sensor, bool lighthouse) {
//    TRIANGULATED = 0,      // for each sensor
//    DISTANCE = 1,           // for each sensor and lighthouse
//    RAY = 2,   // for each sensor and lighthouse
//    SENSOR_NAME = 3,   // for each sensor
//    DISTANCES = 4
    int n_sensors = sensors.size(), per_lighthouse = n_sensors * NUMBER_OF_LIGHTHOUSES * lighthouse;

    sensor += trackedObjectInstance * 6667;
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
    lighthouse_angles(VERTICAL) += calibration[lighthouse][VERTICAL].phase;
    lighthouse_angles(HORIZONTAL) += calibration[lighthouse][HORIZONTAL].phase;
    lighthouse_angles(VERTICAL) += calibration[lighthouse][VERTICAL].curve *
                                   pow(cos(lighthouse_angles(HORIZONTAL)) * sin(lighthouse_angles(VERTICAL)), 2.0)
                                   + calibration[lighthouse][VERTICAL].gibmag *
                                     cos(lighthouse_angles(VERTICAL) + calibration[lighthouse][VERTICAL].gibphase);
    lighthouse_angles(HORIZONTAL) += calibration[lighthouse][HORIZONTAL].curve *
                                     pow(-sin(lighthouse_angles(HORIZONTAL)) * cos(lighthouse_angles(VERTICAL)), 2.0)
                                     + calibration[lighthouse][HORIZONTAL].gibmag *
                                       cos(lighthouse_angles(HORIZONTAL) +
                                           calibration[lighthouse][HORIZONTAL].gibphase);

}

void LighthouseEstimator::applyCalibrationData(bool lighthouse, double &elevation, double &azimuth) {
    elevation += calibration[lighthouse][VERTICAL].phase;
    azimuth += calibration[lighthouse][HORIZONTAL].phase;
    elevation += calibration[lighthouse][VERTICAL].curve *
                 pow(cos(azimuth) * sin(elevation), 2.0)
                 + calibration[lighthouse][VERTICAL].gibmag *
                   cos(elevation + calibration[lighthouse][VERTICAL].gibphase);
    azimuth += calibration[lighthouse][HORIZONTAL].curve *
               pow(-sin(azimuth) * cos(elevation), 2.0)
               + calibration[lighthouse][HORIZONTAL].gibmag *
                 cos(azimuth + calibration[lighthouse][HORIZONTAL].gibphase);
}

MatrixXd LighthouseEstimator::Pinv(MatrixXd A) {

    int n_rows = A.rows();
    int n_cols = A.cols();
    //MatrixXf A_pinv = MatrixXf::Zero(n_cols,n_rows);
    JacobiSVD<MatrixXd> svd(A, ComputeFullU | ComputeFullV);
    if (n_rows < n_cols) {
        VectorXd S = svd.singularValues();
        MatrixXd S_pinv = MatrixXd::Zero(n_cols, n_rows);
        for (int i = 0; i < n_rows; i++) {
            if (S(i) > 1e-6) {
                S_pinv(i, i) = 1 / S(i);
            }
        }
        return svd.matrixV() * S_pinv * svd.matrixU().transpose();
    } else {
        VectorXd S = svd.singularValues();
        MatrixXd S_pinv = MatrixXd::Zero(n_cols, n_rows);
        for (int i = 0; i < n_cols; i++) {
            if (S(i) > 1e-6) {
                S_pinv(i, i) = 1 / S(i);
            }
        }
        return svd.matrixV() * S_pinv * svd.matrixU().transpose();
    }
}