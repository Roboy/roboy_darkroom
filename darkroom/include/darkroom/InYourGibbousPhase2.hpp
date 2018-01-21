#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// std
#include <iostream>
#include "darkroom/Sensor.hpp"

using namespace Eigen;
using namespace std;

namespace InYourGibbousPhase2 {
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

    struct InYourGibbousPhase2 : Functor<double> {
        /**
         * Default amount of sensors needed for Eigen templated structure
         * @param numberOfSensors you can however choose any number of sensors here
         */
        InYourGibbousPhase2(int trajectoryPoints = 4);

        /**
         * This is the function that is called in each iteration
         * @param x contains the calibration values [phase0 phase1 tilt0 tilt1 curve0 curve1 gibphase0 gibphase1 gibmag0 gibmag1]
         * @param fvec the error function (the difference between the sensor positions)
         * @return
         */
        int operator()(const VectorXd &x, VectorXd &fvec) const;

        vector<vector<Vector2d>> angles_measured_trajectory;
        vector<vector<Vector3d>> relative_positions_trajectory;
        double distance_y = 2.0;
        int trajectoryPoints = 4;
        bool lighthouse;
        enum {phase_horizontal , phase_vertical, tilt_horizontal, tilt_vertical, curve_horizontal, curve_vertical,
            gibphase_horizontal, gibphase_vertical, gibmag_horizontal, gibmag_vertical};
    };
}