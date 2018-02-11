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
    description: least squares minimzer for pose estimation using relative sensor distances
                  based on "An Improved Method of Pose Estimation for Lighthouse Base Station Extension"
                  https://www.ncbi.nlm.nih.gov/pmc/articles/PMC5677447/
*/

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// std
#include <iostream>

using namespace Eigen;
using namespace std;
namespace InYourGibbousPhase4 {
// Generic functor for Eigen Levenberg-Marquardt minimizer
    template<typename _Scalar, int NX = Dynamic, int NY = Dynamic>
    struct Functor {
        typedef _Scalar Scalar;
        enum {
            InputsAtCompileTime = NX,
            ValuesAtCompileTime = NY
        };
        typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
        typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
        typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

        const int m_inputs, m_values;

        Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}

        Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

        int inputs() const { return m_inputs; }

        int values() const { return m_values; }
    };

    struct PoseEstimator : Functor<double> {
        /**
         * Default amount of sensors needed for Eigen templated structure
         * @param numberOfSensors you can however choose any number of sensors here
         */
        PoseEstimator(int numberOfSensors = 4);

        /**
         * This is the function that is called in each iteration
         * @param x the pose vector (3 rotational 3 translational parameters)
         * @param fvec the error function (the difference between the sensor positions)
         * @return
         */
        int operator()(const VectorXd &x, VectorXd &fvec) const;

        VectorXd pose;
        vector<Matrix4d> lighthousePose;
        vector<double> elevations, azimuths;
        vector<Vector4d> rel_pos;
        int numberOfSensors = 4;
        vector<int> lighthouse_id;
        enum {curve_horizontal = 6, curve_vertical = 7,
            gibphase_horizontal = 8, gibphase_vertical = 9, gibmag_horizontal = 10, gibmag_vertical = 11};
    };
}