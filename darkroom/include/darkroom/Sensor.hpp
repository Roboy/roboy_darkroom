#pragma once

// ros
#include <ros/ros.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// std
#include <mutex>
#include <vector>
#include <chrono>

#define NUMBER_OF_LIGHTHOUSES 2

using namespace Eigen;
using namespace std;
using namespace chrono;

enum ANGLE_TYPE{
    HORIZONTAL = 0,
    VERTICAL = 1
};

#define LIGHTHOUSE_A false
#define LIGHTHOUSE_B true

class Sensor{
public:
    Sensor();

    /**
     * Updates the angles for the respective lighthouse
     * @param lighthouse for which lighthouse
     * @param type #ANGLE_TYPE
     * @param angle ligthhouse angle
     */
    void update(bool lighthouse, int type, double angle);

    /**
     * Switches lighthouses, such that angles for lighthouse 1 belong to lighthouse 2
     * @param flag true/false
     */
    void switchLighthouses(bool flag);

    /**
     * Returns if the lighthouses are switched
     * @return true/false
     */
    bool getSwitchLighthouses();

    /**
     * Get the recent angles
     * @param lighthouse of this lighthouse
     * @param angles Vector(VERTICAL, HORIZONTAL)
     */
    void get(bool lighthouse, Vector2d &angles);

    /**
     * Get the recent angles and timestamps
     * @param lighthouse of this lighthouse
     * @param angles Vector(VERTICAL, HORIZONTAL)
     * @param timestamps
     */
    void get(bool lighthouse, Vector2d &angles, high_resolution_clock::time_point *timestamps);

    /**
     * Get the recent angles
     * @param lighthouse of this lighthouse
     * @param elevation angle
     * @param azimuth angle
     */
    void get(bool lighthouse, double &elevation, double &azimuth);

    /**
     * pushes the recent angles into the vectors
     * @param lighthouse of this lighthouse
     * @param elevations vector<angle>
     * @param azimuths vector<angle>
     */
    void get(bool lighthouse, vector<double> &elevations, vector<double> &azimuths);

    /**
     * Set the relative 3d position, wrt to a lighthouse
     * @param lighthouse
     * @param position3D
     */
    void set(bool lighthouse, Vector3d &position3D);

    /**
     * Gets the 3d position of a sensor
     * @param position3D
     * @return
     */
    bool getPosition3D(Vector3d &position3D);
    /**
     * Gets the relative 3d position of a sensor wrt to a lighthouse
     * @param lighthouse
     * @param position3D
     * @return
     */
    bool get(bool lighthouse, Vector3d &position3D);

    /**
     * Set the absolute 3d position, wrt to a world frame
     * @param lighthouse
     * @param position3D
     */
    void set(Vector3d &position3D);

    /**
     * Returns the euclidean distance to a lighthouse in meter
     * @param lighthouse
     * @return
     */
    double getDistance(bool lighthouse);

    /**
     * A sensor is considered active, when it has been updated less than a certain amount of time ago
     * @param lighthouse
     * @return active/inactive
     */
    bool isActive(bool lighthouse);

    /**
     * Checks if the provided timestamp matches the current timestamp
     * @param timestamp four timestamps, one for each lighthouse and rotor
     * @return new data available
     */
    bool hasNewData(high_resolution_clock::time_point *timestamp);

    /**
     * Sets the sensors relative location on the tracked object
     * @param relative_location the relative 3D position
     */
    void setRelativeLocation(Vector3d &relative_location);

    /**
     * Gets the sensors relative location on the tracked object
     * @param relative_location the relative 3D position
     */
    void getRelativeLocation(Vector3d &relative_location);
    /**
     * Gets the sensors relative location on the tracked object
     * @param relative_location the relative 3D position in homogenous coordinates
     */
    void getRelativeLocation(Vector4d &relative_location);

    /**
     * Indicates if the sensors relative location is known
     * @return true/false
     */
    bool isCalibrated();

    /**
     * pushes the sensors relative location on the tracked object
     * @param relative_locations into this vector
     */
    void getRelativeLocation(vector<Vector3d> &relative_locations);

    /**
     * Estimates the angle update frequency from the sensors timestamps
     * @param lighthouse
     * @param horizontal
     * @param vertical
     */
    void updateFrequency(bool lighthouse, float &horizontal, float &vertical);

private:
    static bool m_switch;
    Vector3d m_relative_location;
    Vector3d m_position3D, m_relativePosition3D[NUMBER_OF_LIGHTHOUSES], m_relativeOrigin3D[NUMBER_OF_LIGHTHOUSES];
    double m_angles_horizontal[NUMBER_OF_LIGHTHOUSES], m_angles_vertical[NUMBER_OF_LIGHTHOUSES];
    high_resolution_clock::time_point m_angleUpdateTime_cur[NUMBER_OF_LIGHTHOUSES][2], m_angleUpdateTime_prev[NUMBER_OF_LIGHTHOUSES][2];
    float m_updateFrequency[NUMBER_OF_LIGHTHOUSES][2] = {{0,0}};
    mutex m_lockMutex;
    bool calibrated = false;
};
