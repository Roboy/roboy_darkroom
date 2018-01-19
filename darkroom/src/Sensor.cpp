#include "darkroom/Sensor.hpp"

bool Sensor::m_switch = false;

Sensor::Sensor(){
    m_relativePosition3D[0] = Vector3d(0,1,0);
    m_relativePosition3D[1] = Vector3d(0,1,0);

}

void Sensor::update(bool lighthouse, int type, double angle){
    lock_guard<std::mutex> lock(m_lockMutex);
    if (type == HORIZONTAL) {
        m_angles_horizontal[lighthouse] = angle;
        m_angleUpdateTime_prev[lighthouse][HORIZONTAL] = m_angleUpdateTime_cur[lighthouse][HORIZONTAL];
        m_angleUpdateTime_cur[lighthouse][HORIZONTAL] = high_resolution_clock::now();
    } else {
        m_angles_vertical[lighthouse] = angle;
        m_angleUpdateTime_prev[lighthouse][VERTICAL] = m_angleUpdateTime_cur[lighthouse][VERTICAL];
        m_angleUpdateTime_cur[lighthouse][VERTICAL] = high_resolution_clock::now();
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

void Sensor::get(bool lighthouse, Vector2d &angles, high_resolution_clock::time_point *timestamps){
    lock_guard<std::mutex> lock(m_lockMutex);
    angles = Vector2d(m_angles_vertical[m_switch?!lighthouse:lighthouse],
                      m_angles_horizontal[m_switch?!lighthouse:lighthouse]);
    timestamps[HORIZONTAL] = m_angleUpdateTime_cur[lighthouse][HORIZONTAL];
    timestamps[VERTICAL] = m_angleUpdateTime_cur[lighthouse][VERTICAL];
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
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    microseconds time_span[2] = {duration_cast<microseconds>(t1-m_angleUpdateTime_cur[lighthouse][HORIZONTAL]),
                                 duration_cast<microseconds>(t1-m_angleUpdateTime_cur[lighthouse][VERTICAL])};
//    return true;
    return (time_span[HORIZONTAL].count() > 0 && time_span[HORIZONTAL].count() < 20000 &&
            time_span[VERTICAL].count() > 0 && time_span[VERTICAL].count() < 20000 );
}

bool Sensor::hasNewData(high_resolution_clock::time_point *timestamp){
    return (m_angleUpdateTime_cur[0][HORIZONTAL]!=timestamp[HORIZONTAL] ||
            m_angleUpdateTime_cur[0][VERTICAL]!=timestamp[VERTICAL] ||
            m_angleUpdateTime_cur[1][HORIZONTAL]!=timestamp[2+HORIZONTAL] ||
            m_angleUpdateTime_cur[1][VERTICAL]!=timestamp[2+VERTICAL]);
}

void Sensor::setRelativeLocation(Vector3d &relative_location){
    m_relative_location = relative_location;
    calibrated = true;
}

void Sensor::getRelativeLocation(Vector3d &relative_location){
    relative_location = m_relative_location;
}

void Sensor::getRelativeLocation(Vector4d &relative_location){
    relative_location  << m_relative_location, 1;
}

bool Sensor::isCalibrated(){
    return calibrated;
}

void Sensor::getRelativeLocation(vector<Vector3d> &relative_locations){
    relative_locations.push_back(m_relative_location);
}

void Sensor::updateFrequency(bool lighthouse, float &horizontal, float &vertical){
    microseconds update_period[2] = {duration_cast<microseconds>(m_angleUpdateTime_cur[lighthouse][HORIZONTAL]-
                                                                         m_angleUpdateTime_prev[lighthouse][HORIZONTAL]),
                                 duration_cast<microseconds>(m_angleUpdateTime_cur[lighthouse][VERTICAL]-
                                                             m_angleUpdateTime_prev[lighthouse][VERTICAL])};
    // convert millisecond period to Hz
    if(update_period[HORIZONTAL].count()>0)
        m_updateFrequency[lighthouse][HORIZONTAL] = 0.5f*1000000.0f/update_period[HORIZONTAL].count() +
                0.5f*m_updateFrequency[lighthouse][HORIZONTAL];
    else
        m_updateFrequency[lighthouse][HORIZONTAL] = 0.5f*m_updateFrequency[lighthouse][HORIZONTAL];

    if(update_period[VERTICAL].count()>0)
        m_updateFrequency[lighthouse][VERTICAL] = 0.5f*1000000.0f/update_period[VERTICAL].count() +
                                                    0.5f*m_updateFrequency[lighthouse][VERTICAL];
    else
        m_updateFrequency[lighthouse][VERTICAL] = 0.5f*m_updateFrequency[lighthouse][VERTICAL];

    horizontal = m_updateFrequency[lighthouse][HORIZONTAL];
    vertical = m_updateFrequency[lighthouse][VERTICAL];
}
