#pragma once

#include <tf/tf.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

// std
#include <iostream>

#include "darkroom/Triangulation.hpp"

using namespace Eigen;
using namespace std;

namespace PoseEstimatorSensorDistance {

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
         * @param numberOfSamples the pose will be estimated using this amount of samples
         * @param distanceBetweenSensors the distance between the two sensors in mm
         */
        PoseEstimator(int numberOfSamples, double distanceBetweenSensors, MatrixXd &rays0_A, MatrixXd &rays0_B,
                      MatrixXd &rays1_A, MatrixXd &rays1_B);

        /**
         * This is the function that is called in each iteration
         * @param x the pose vector (3 rotational 3 translational parameters)
         * @param fvec the error function (the difference between the sensor positions)
         * @return
         */
        int operator()(const VectorXd &x, VectorXd &fvec) const;

        VectorXd pose;
        MatrixXd rays0_A, rays0_B, rays1_A, rays1_B; // rays of each lighthouse pointing to the sensors
        double distanceBetweenSensors;
        int numberOfSamples;
        Matrix4d RT_A, RT_B;
        static int counter;
    };

}