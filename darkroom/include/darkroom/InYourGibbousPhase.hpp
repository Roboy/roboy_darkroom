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

enum {phase0 , phase1, tilt0, tilt1, curve0, curve1, gibphase0, gibphase1, gibmag0, gibmag1};

namespace InYourGibbousPhase {
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

    struct InYourGibbousPhase : Functor<double> {
        /**
         * Default amount of sensors needed for Eigen templated structure
         * @param numberOfSensors you can however choose any number of sensors here
         */
        InYourGibbousPhase(int numberOfSensors = 4);

        /**
         * This is the function that is called in each iteration
         * @param x contains the calibration values [phase0 phase1 tilt0 tilt1 curve0 curve1 gibphase0 gibphase1 gibmag0 gibmag1]
         * @param fvec the error function (the difference between the sensor positions)
         * @return
         */
        int operator()(const VectorXd &x, VectorXd &fvec) const;

        vector<double> elevation_measured, elevation_truth;
        vector<double> azimuth_measured, azimuth_truth;
        int numberOfSensors = 4;
        bool lighthouse;
    };
}