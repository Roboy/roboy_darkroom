#include "darkroom/Sensor.hpp"

bool Sensor::m_switch = false;

Sensor::Sensor(){
    m_relativePosition3D[0] = Vector3d(0,1,0);
    m_relativePosition3D[1] = Vector3d(0,1,0);
}

void Sensor::update(bool lighthouse, int type, unsigned short timestamp, double angle){
    lock_guard<std::mutex> lock(m_lockMutex);
    if (type == HORIZONTAL) {
        m_angles_horizontal[lighthouse] = make_pair(timestamp, angle);
    } else {
        m_angles_vertical[lighthouse] = make_pair(timestamp, M_PI-angle);
    }
    m_angleUpdateTime[lighthouse] = ros::Time::now();
}

void Sensor::switchLighthouses(bool switchID){
    lock_guard<std::mutex> lock(m_lockMutex);
    m_switch = switchID;
}

bool Sensor::getSwitchLighthouses(){
    return m_switch;
}

void Sensor::get(bool lighthouse, Vector2d &angles){
    lock_guard<std::mutex> lock(m_lockMutex);
    angles = Vector2d(m_angles_vertical[m_switch?!lighthouse:lighthouse].second,
                      m_angles_horizontal[m_switch?!lighthouse:lighthouse].second);
}

void Sensor::get(bool lighthouse, Vector2d &angles, ros::Time *timestamps){
    lock_guard<std::mutex> lock(m_lockMutex);
    angles = Vector2d(m_angles_vertical[m_switch?!lighthouse:lighthouse].second,
                      m_angles_horizontal[m_switch?!lighthouse:lighthouse].second);
    timestamps[0] = m_angleUpdateTime[m_switch?!lighthouse:lighthouse];
    timestamps[1] = m_angleUpdateTime[m_switch?lighthouse:!lighthouse];
}

void Sensor::get(bool lighthouse, double &elevation, double &azimuth){
    elevation = m_angles_vertical[m_switch?!lighthouse:lighthouse].second;
    azimuth = m_angles_horizontal[m_switch?!lighthouse:lighthouse].second;
}

void Sensor::get(bool lighthouse, vector<double> &elevations, vector<double> &azimuths){
    elevations.push_back(m_angles_vertical[m_switch?!lighthouse:lighthouse].second);
    azimuths.push_back(m_angles_horizontal[m_switch?!lighthouse:lighthouse].second);
}

void Sensor::set(bool lighthouse, Vector3d &position3D){
    m_relativePosition3D[m_switch?!lighthouse:lighthouse] = position3D;
    m_relativeOrigin3D[m_switch?!lighthouse:lighthouse] = position3D;
}

bool Sensor::get(bool lighthouse, Vector3d &position3D){
    position3D = m_relativePosition3D[m_switch?!lighthouse:lighthouse];
    return true;
}

bool Sensor::getPosition3D(Vector3d &position3D){
    position3D = m_position3D;
    return true;
}

void Sensor::set(Vector3d &position3D){
    m_position3D = position3D;
}

double Sensor::getDistance(bool lighthouse){
    return m_relativePosition3D[m_switch?!lighthouse:lighthouse].norm();
}

bool Sensor::isActive(bool lighthouse){
    return (ros::Time::now() - m_angleUpdateTime[m_switch?!lighthouse:lighthouse]) < ros::Duration(1.0);
}

void Sensor::setRelativeLocation(Vector3d &relative_location){
    m_relative_location = relative_location;
    calibrated = true;
}

void Sensor::getRelativeLocation(Vector3d &relative_location){
    relative_location = m_relative_location;
}

bool Sensor::sensorCalibrated(){
    return calibrated;
}

void Sensor::getRelativeLocation(vector<Vector3d> &relative_locations){
    relative_locations.push_back(m_relative_location);
}
