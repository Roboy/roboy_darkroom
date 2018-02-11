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
    description: helper class for handling triangulation from lighthouse angles
*/

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include "darkroom/Sensor.hpp"

struct LighthouseCalibration{
    double phase = 0;
    double curve = 0;
    double tilt = 0;
    double gibmag = 0;
    double gibphase = 0;
    void reset(){ phase = 0; curve = 0; tilt = 0; gibmag = 0; gibphase = 0; }
};

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

    void rayFromLighthouseAngles(Vector2d &angles, Vector3d &ray);
    void rayFromLighthouseAngles(double elevation, double azimuth, Vector3d &ray);

    // for each motor and each lighthouse
    LighthouseCalibration calibration[2][2];
};