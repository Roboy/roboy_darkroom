#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include "darkroom/Sensor.hpp"

// offset from center of laser rotation
#define AXIS_OFFSET 0.015

using namespace Eigen;

class Triangulation{
public:
    /**
   * get the 3D minimum distance between 2 lines
   * following http://geomalgorithms.com/a07-_distance.html#Distance-between-Lines
   * @param pos0 origin line0
   * @param dir1 direction line0
   * @param pos1 origin line1
   * @param dir2 direction line1
   * @param tri0 point on line0 closest to line1
   * @param tri1 point on line1 closest to line0
   * @return distance between the lines
   */
    double dist3D_Line_to_Line( Vector3d &pos0, Vector3d &dir1,
                                Vector3d &pos1, Vector3d &dir2,
                                Vector3d &tri0, Vector3d &tri1);

/**
    * This function triangulates the position of a sensor using the horizontal and vertical angles from two ligthouses
    * @param angles0 vertical/horizontal angles form first lighthouse
    * @param angles1 vertical/horizontal angles form second lighthouse
    * @param RT_0 pose matrix of first lighthouse
    * @param RT_1 pose matrix of second lighthouses
    * @param triangulated_position the triangulated position
    * @param ray0 ligthhouse ray
    * @param ray1 ligthhouse ray
    */
    double triangulateFromLighthouseAngles(Vector2d &angles0, Vector2d &angles1, Matrix4d &RT_0, Matrix4d &RT_1,
                                           Vector3d &triangulated_position, Vector3d &ray0, Vector3d &ray1);

    double triangulateFromRays(Vector3d &ray0, Vector3d &ray1, Matrix4d &RT_0, Matrix4d &RT_1, Vector3d &triangulated_position);

    void rayFromLighthouseAngles(Vector2d &angles, Vector3d &ray, bool lighthouse);

    // for each motor and each lighthouse
    double phase[2][2] = {{0,0},{0,0}}, tilt[2][2] = {{0,0},{0,0}}, curve[2][2] = {{0,0},{0,0}},
            gibphase[2][2] = {{0,0},{0,0}}, gibmag[2][2] = {{0,0},{0,0}};
};