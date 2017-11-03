#include "darkroom/Transform.hpp"
#include "darkroom/Triangulation.hpp"
#include "darkroom/PoseEstimatorSensorCloud.hpp"
#include "darkroom/PoseEstimatorSensorDistance.hpp"
#include "darkroom/PoseEstimatorSensorDistances.hpp"
#include "darkroom/ParticleFilter.hpp"
#include "darkroom/Sensor.hpp"
#include <common_utilities/rviz_visualization.hpp>
#include <roboy_communication_middleware/LighthousePoseCorrection.h>
#include <roboy_communication_middleware/DarkRoomSensor.h>

#include <darkroom/epnp/epnp.h>
//#include <darkroom/mavmap/src/base3d/p3p.h>

#include <atomic>
#include <mutex>

#define LIGHTHOUSE_A false
#define LIGHTHOUSE_B true

#define MAX_ITERATIONS 100
#define ERROR_THRESHOLD 0.0001

#define NUMBER_OF_SAMPLES 100

#define NUMBER_OF_PARTICLES 1000

static vector<int> DEFAULT_VECTOR;

class LighthouseEstimator:public DarkRoom::Transform, public rviz_visualization{
public:
    LighthouseEstimator();

    /**
     * Estimates the pose correction between ligthhouse 1 and 2, such that the squared distances between sensor positions
     * estimated for both lighthouses is minimized.
     * @return success
     */
    bool lighthousePoseEstimationLeastSquares();

    bool objectPoseEstimationLeastSquares();

    /**
    * Estimates the sensor distances of all active sensors (or a vector of specified sensor ids)
    * using the known (ie. calibrated) relative distance between the sensors and the lighthouse angles
    * @param lighthouse for which lighthouse
    * @param specificIds if defined, waits until the specified sensors become active
    * @return
    */
    bool estimateSensorPositionsUsingRelativeDistances(bool lighthouse, vector<int> &specificIds = DEFAULT_VECTOR);

    bool poseEstimationSensorDistance();
    bool poseEstimationSensorDistances();
    bool poseEstimationP3P();
    bool poseEstimationEPnP();
    bool poseEstimationParticleFilter();

    /**
     * Triangulates the sensor positions (the transform between lighthouse 1 and 2 needs to be known, otherwise the
     * triangulated position is not correct)
     */
    void triangulateSensors();

    /**
     * Measures triangulated sensor locations for 30 seconds. Calculates mean sensor locations and generates
     * relative sensor positions which are saved to a yaml file
     */
    void calibrateRelativeSensorDistances();

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

    enum POSE_CORRECTION_TYPE{
        RELATIV = 0,
        ABSOLUT = 1,
        OBJECT = 2
    };

    map<int, Sensor> sensors;
    vector<int> calibrated_sensors;
    atomic<bool> tracking, calibrating, poseestimating, objectposeestimating,
            distances, rays, particle_filtering;
    mutex mux;
private:
    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    ros::Publisher sensor_location_pub, lighthouse_pose_correction;
    VectorXd object_pose;
};
