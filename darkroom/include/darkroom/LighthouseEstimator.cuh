#pragma once


#pragma hd_warning_disable
#include <cuda_runtime.h>
#include <string>

using std::string;

//! cuda error checking
void cuda_check(string file, int line);
#define CUDA_CHECK cuda_check(__FILE__,__LINE__)

#include "darkroom/Transform.hpp"
#include "darkroom/Triangulation.hpp"
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
#include <ros/package.h>
#include "darkroom/Utilities.hpp"

namespace fs = boost::filesystem;

#define MAX_ITERATIONS 100
#define ERROR_THRESHOLD 0.00001

#define NUMBER_OF_SAMPLES 100

#define NUMBER_OF_PARTICLES 1000

static vector<int> DEFAULT_VECTOR2;

class LighthouseEstimatorCUDA : public DarkRoom::Transform, public rviz_visualization, public Triangulation, public Utilities {
public:
    LighthouseEstimatorCUDA();

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
    * Estimates the sensor distances of all active sensors (or a vector of specified sensor ids)
    * using the known (ie. calibrated) relative distance between the sensors and the lighthouse angles
    * @param lighthouse for which lighthouse
    * @param specificIds if defined, waits until the specified sensors become active
    * @return
    */
    bool estimateSensorPositionsUsingRelativeDistances(bool lighthouse, vector<int> &specificIds = DEFAULT_VECTOR2);

    /**
     * Triangulates the sensor positions (the transform between lighthouse 1 and 2 needs to be known, otherwise the
     * triangulated position is not correct)
     */
    void triangulateSensors();

    /**
     * Publishes the lighthouse rays
     */
    void publishRays();

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
    map<int,vector<double>> calibration_angles;
    int active_sensors = 0;
    atomic<bool> tracking, calibrating, poseestimating, objectposeestimating,
            distances, rays, particle_filtering, use_lighthouse_calibration_data_phase[2],
            use_lighthouse_calibration_data_tilt[2], use_lighthouse_calibration_data_gibphase[2],
            use_lighthouse_calibration_data_gibmag[2];
    mutex mux;
    fs::path mesh;
    bool has_mesh = false;
    string name = "bastiisdoff";
    string imu_topic_name, pose_topic_name;
    ros::Publisher pose_pub;
    tf::Transform pose;
private:
    void applyCalibrationData(Vector2d &lighthouse0_angles, Vector2d &lighthouse1_angles);
    void applyCalibrationData(bool lighthouse, Vector2d &lighthouse_angles);
    void applyCalibrationData(bool lighthouse, double &elevation, double &azimuth);
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher sensor_location_pub, lighthouse_pose_correction;
    ros::Subscriber ootx_sub;
    VectorXd object_pose;
    OOTXframe ootx[2];
};
