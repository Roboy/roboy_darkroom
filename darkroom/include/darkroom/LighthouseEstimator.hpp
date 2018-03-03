/*
    BSD 3-Clause License

    Copyright (c) 2018, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: triangulation, pose estimation, lighthouse pose correction, factory calibration
*/

#include "darkroom/Transform.hpp"
#include "darkroom/Triangulation.hpp"
#include "darkroom/PoseEstimatorSensorCloud.hpp"
#include "darkroom/PoseEstimatorMultiLighthouse.hpp"
#include "darkroom/Sensor.hpp"
#include <common_utilities/rviz_visualization.hpp>
#include <common_utilities/CommonDefinitions.h>
#include <roboy_communication_middleware/LighthousePoseCorrection.h>
#include <roboy_communication_middleware/DarkRoomSensor.h>
#include <roboy_communication_middleware/DarkRoomOOTX.h>
#include <roboy_communication_middleware/ArucoPose.h>
#include <common_utilities/CommonDefinitions.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/filesystem.hpp>
#include <atomic>
#include <mutex>
#include "darkroom/InYourGibbousPhase.hpp"
#include "darkroom/InYourGibbousPhase2.hpp"
#include "darkroom/InYourGibbousPhase3.hpp"
#include "darkroom/InYourGibbousPhase4.hpp"
#include <ros/package.h>
#include "darkroom/Utilities.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <optimization.h>
#include <epnp/epnp.h>

using namespace alglib;

namespace fs = boost::filesystem;

#define MAX_ITERATIONS 100
#define ERROR_THRESHOLD 0.0000001

#define NUMBER_OF_SAMPLES 100

#define NUMBER_OF_PARTICLES 1000

static vector<int> DEFAULT_VECTOR;

void function1_fvec(const real_1d_array &x, real_1d_array &fi, void *ptr);

class LighthouseEstimator
        : public DarkRoom::Transform, public rviz_visualization, public Triangulation, public Utilities {
public:
    LighthouseEstimator();

    /**
     * This returns the sensors that are calibrated and visible by both lighthouses
     * @param visible_sensors will be filled with sensor ids
     */
    void getVisibleCalibratedSensors(vector<int> &visible_sensors);

    /**
     * This returns the sensors that are calibrated and visible to the given lighthouse
     * @param visible_sensors will be filled with sensor ids
     */
    void getVisibleCalibratedSensors(bool lighthouse, vector<int> &visible_sensors);

    /**
     * Estimates the pose correction between ligthhouse 1 and 2, such that the squared distances between sensor positions
     * estimated for both lighthouses is minimized.
     * @return success
     */
    bool lighthousePoseEstimationLeastSquares();

    /**
     * Estimates the pose of an object using relative sensor position information and least square matching
     */
    void objectPoseEstimationLeastSquares();

    /**
    * Estimates the sensor distances of all active sensors (or a vector of specified sensor ids)
    * using the known (ie. calibrated) relative distance between the sensors and the lighthouse angles
    * @param lighthouse for which lighthouse
    * @param specificIds if defined, waits until the specified sensors become active
    * @return
    */
    bool estimateSensorPositionsUsingRelativeDistances(bool lighthouse, vector<int> &specificIds = DEFAULT_VECTOR);

    /**
    * Estimates the sensor distances of all active sensors
    * using the known (ie. calibrated) relative distance between the sensors and the lighthouse angles, then estimates
    * the object pose relative to each ligthhouse
    * @return success
    */
    bool estimateObjectPoseUsingRelativeDistances();

    /**
     * Estimates object pose using multi lighthouse approach
     */
    void estimateObjectPoseMultiLighthouse();
    /**
     * Triangulates the sensor positions (the transform between lighthouse 1 and 2 needs to be known, otherwise the
     * triangulated position is not correct)
     */
    void triangulateSensors();

    /**
     * Estimates relative object pose using epnp
     */
    void estimateObjectPoseEPNP();
    /**
     * Publishes the lighthouse rays
     */
    void publishRays();

    /**
     * Measures triangulated sensor locations for 30 seconds. Calculates mean sensor locations and generates
     * relative sensor positions which are saved to a yaml file
     */
    void calibrateRelativeSensorDistances();

    /**
     * Estimates calibration values based on known sensor angles
     * @param lighthouse for this lighthouse
     * @return success
     */
    bool estimateFactoryCalibration(int lighthouse);

    /**
     * Estimates calibration values using epnp
     * @param lighthouse for this lighthouse
     * @return success
     */
    bool estimateFactoryCalibrationEPNP(int lighthouse);

    /**
     * Estimates calibration values using multi lighthouse pose estimator
     * @param lighthouse for this lighthouse
     * @return success
     */
    bool estimateFactoryCalibrationMultiLighthouse(int lighthouse);

    /**
     * Estimates calibration values based on known sensor angles
     * @param lighthouse for this lighthouse
     * @return success
     */
    bool estimateFactoryCalibration2(int lighthouse);

    /**
     * Returns a unique id for #MESSAGE_ID sensor and lighthouse
     * @param type the message type #MESSAGE_ID
     * @param sensor the sensor id
     * @param lighthouse the lighthouse
     * @return a unique id
     */
    int getMessageID(int type, int sensor, bool lighthouse = false);

    enum MESSAGE_ID {
        TRIANGULATED = 0,      // for each sensor
        DISTANCE = 1,           // for each sensor and lighthouse
        RAY = 2,   // for each sensor and lighthouse
        SENSOR_NAME = 3,   // for each sensor
        DISTANCES = 4
    };

    enum POSE_CORRECTION_TYPE {
        RELATIV = 0,
        ABSOLUT = 1,
        OBJECT = 2
    };

    map<int, Sensor> sensors;
    vector<int> calibrated_sensors;
    map<int, vector<double>> calibration_angles;
    int active_sensors = 0;
    atomic<bool> tracking, calibrating, poseestimating, poseestimating_epnp, poseestimating_multiLighthouse, objectposeestimating,
            distances, rays, comparesteamvr;
    SharedMutex mux;
    fs::path mesh;
    bool has_mesh = false;
    string name = "bastiisdoff";
    string imu_topic_name, pose_topic_name;
    ros::Publisher pose_pub;
    tf::Transform pose;
    ofstream steamVRrecord;
    static int trackedObjectInstance; //! a unique object instance (helps with unique rviz marker ids)
private:
    void receiveOOTXData(const roboy_communication_middleware::DarkRoomOOTX::ConstPtr &msg);

    void applyCalibrationData(Vector2d &lighthouse0_angles, Vector2d &lighthouse1_angles);

    void applyCalibrationData(bool lighthouse, Vector2d &lighthouse_angles);

    void applyCalibrationData(bool lighthouse, double &elevation, double &azimuth);

    MatrixXd Pinv(MatrixXd A);
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher sensor_location_pub, lighthouse_pose_correction;
    ros::Subscriber ootx_sub;
    VectorXd object_pose;
    OOTXframe ootx[2];
};
