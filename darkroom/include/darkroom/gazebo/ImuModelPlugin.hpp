#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <chrono>

using namespace Eigen;
using namespace gazebo;
using namespace std;
using namespace chrono;

class ImuModelPlugin : public gazebo::ModelPlugin{
public:
    /** Constructor */
    ImuModelPlugin();
    /** Destructor */
    ~ImuModelPlugin();

    /**
     * Overloaded Gazebo entry point
     * @param parent model pointer
     * @param sdf element
     */
    void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

    /** Called at each sim step */
    void Update();

    /**
     * Read from Simulation
     * @param time current time
     * @param period period since last read
     */
    void readSim(ros::Time time, ros::Duration period);

    /** Write to Simulation
     * @param time current time
     * @param period period since last read
     */
    void writeSim(ros::Time time, ros::Duration period);

    /** Called on world reset */
    void Reset();

private:
    enum{
        ACC = 1,
        GYRO = 2,
        IMU = ACC|GYRO
    };

    struct ImuInfo{
        gazebo::physics::LinkPtr link;
        Vector3d position;
        Vector3d orientation;
        int type = IMU;
    };

    bool parseImuSDF(const string &sdf, vector<ImuInfo> &imu);

    vector<ImuInfo> imus;
    vector<ros::Publisher> imu_pub;

    ros::NodeHandlePtr nh;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    gazebo::physics::ModelPtr parent_model;
    sdf::ElementPtr sdf;
    vector<string> link_names;
    // Pointer to the update event connection
    gazebo::event::ConnectionPtr update_connection;
    // Timing
    gazebo::common::Time gz_time_now;
    gazebo::common::Time gz_period;
    gazebo::common::Time gz_last;
    gazebo::common::Time gz_last_write_sim_time_ros;

    ros::Time last_update_sim_time_ros;
    ros::Time last_write_sim_time_ros;

    high_resolution_clock::time_point t0,t1;
};
