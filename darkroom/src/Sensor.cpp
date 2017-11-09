#include "darkroom/Sensor.hpp"

bool Sensor::m_switch = false;

Sensor::Sensor(){
    m_relativePosition3D[0] = Vector3d(0,1,0);
    m_relativePosition3D[1] = Vector3d(0,1,0);

}

void Sensor::update(bool lighthouse, int type, int timestamp, double angle){
    lock_guard<std::mutex> lock(m_lockMutex);
    if (type == HORIZONTAL) {
        m_angles_horizontal[lighthouse] = angle;
        m_angleUpdateTime_prev[lighthouse][HORIZONTAL] = m_angleUpdateTime_current[lighthouse][HORIZONTAL];
        m_angleUpdateTime_current[lighthouse][HORIZONTAL] = timestamp;
    } else {
        m_angles_vertical[lighthouse] = M_PI-angle;
        m_angleUpdateTime_prev[lighthouse][VERTICAL] = m_angleUpdateTime_current[lighthouse][VERTICAL];
        m_angleUpdateTime_current[lighthouse][VERTICAL] = timestamp;
    }
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
    angles = Vector2d(m_angles_vertical[m_switch?!lighthouse:lighthouse],
                      m_angles_horizontal[m_switch?!lighthouse:lighthouse]);
}

void Sensor::get(bool lighthouse, Vector2d &angles, int *timestamps){
    lock_guard<std::mutex> lock(m_lockMutex);
    angles = Vector2d(m_angles_vertical[m_switch?!lighthouse:lighthouse],
                      m_angles_horizontal[m_switch?!lighthouse:lighthouse]);
    timestamps[HORIZONTAL] = m_angleUpdateTime_current[m_switch?!lighthouse:lighthouse][HORIZONTAL];
    timestamps[VERTICAL] = m_angleUpdateTime_current[m_switch?lighthouse:!lighthouse][VERTICAL];
}

void Sensor::get(bool lighthouse, double &elevation, double &azimuth){
    elevation = m_angles_vertical[m_switch?!lighthouse:lighthouse];
    azimuth = m_angles_horizontal[m_switch?!lighthouse:lighthouse];
}

void Sensor::get(bool lighthouse, vector<double> &elevations, vector<double> &azimuths){
    elevations.push_back(m_angles_vertical[m_switch?!lighthouse:lighthouse]);
    azimuths.push_back(m_angles_horizontal[m_switch?!lighthouse:lighthouse]);
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
    int period[2] = {m_angleUpdateTime_current[m_switch?!lighthouse:lighthouse][HORIZONTAL] -
                     m_angleUpdateTime_prev[m_switch?!lighthouse:lighthouse][HORIZONTAL],
                     m_angleUpdateTime_current[m_switch?!lighthouse:lighthouse][VERTICAL] -
                             m_angleUpdateTime_prev[m_switch?!lighthouse:lighthouse][VERTICAL]};
    return (period[HORIZONTAL] > 0 && period[HORIZONTAL] < 50000 &&
            period[VERTICAL] > 0 && period[VERTICAL] < 50000 );
}

void Sensor::setRelativeLocation(Vector3d &relative_location){
    m_relative_location = relative_location;
    calibrated = true;
}

void Sensor::getRelativeLocation(Vector3d &relative_location){
    relative_location = m_relative_location;
}

bool Sensor::isCalibrated(){
    return calibrated;
}

void Sensor::getRelativeLocation(vector<Vector3d> &relative_locations){
    relative_locations.push_back(m_relative_location);
}

void Sensor::updateFrequency(bool lighthouse, float &horizontal, float &vertical){
    int period[2] = {m_angleUpdateTime_current[m_switch?!lighthouse:lighthouse][HORIZONTAL] -
                             m_angleUpdateTime_prev[m_switch?!lighthouse:lighthouse][HORIZONTAL],
                     m_angleUpdateTime_current[m_switch?!lighthouse:lighthouse][VERTICAL] -
                             m_angleUpdateTime_prev[m_switch?!lighthouse:lighthouse][VERTICAL]};
    // convert millisecond period to Hz
    if(period[HORIZONTAL]>0)
        m_updateFrequency[lighthouse][HORIZONTAL] = 0.5f*1000000.0f/period[HORIZONTAL] +
                0.5f*m_updateFrequency[lighthouse][HORIZONTAL];
    else
        m_updateFrequency[lighthouse][HORIZONTAL] = 0.5f*m_updateFrequency[lighthouse][HORIZONTAL];
    if(period[VERTICAL]>0)
        m_updateFrequency[lighthouse][VERTICAL] = 0.5f*1000000.0f/period[VERTICAL] +
                                                    0.5f*m_updateFrequency[lighthouse][VERTICAL];
    else
        m_updateFrequency[lighthouse][VERTICAL] = 0.5f*m_updateFrequency[lighthouse][VERTICAL];

    horizontal = m_updateFrequency[lighthouse][HORIZONTAL];
    vertical = m_updateFrequency[lighthouse][VERTICAL];
}
