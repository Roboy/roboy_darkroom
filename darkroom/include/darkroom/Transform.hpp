#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace Eigen;

namespace DarkRoom {

    class Transform {
    public:
        /**
         * Queries the tf listener for the specified transform
         * @param from we whant the transformation from this frame
         * @param to another frame
         * @param transform the transform if available
         * @return true if available
         */
        bool getTransform(const char *from, const char *to, Matrix4d &transform);

        /**
         * Queries the tf listener for the specified transform
         * @param lighthouse
         * @param to another frame
         * @param transform the transform if available
         * @return true if available
         */
        bool getTransform(bool lighthouse, const char *to, Matrix4d &transform);

        /**
         * Queries the tf listener for the specified transform
         * @param lighthouse
         * @param from another frame
         * @param transform the transform if available
         * @return true if available
         */
        bool getTransform(const char *from, bool lighthouse, Matrix4d &transform);

        /**
         * Queries the tf listener for the specified transform
         * @param to this frame
         * @param from another frame
         * @param transform the transform if available
         * @return true if available
         */
        bool getTransform(const char *from, const char *to, tf::Transform &transform);

        /**
         * Constructs 4x4 RT matrix from 6 pose parameters
         * @param RT filled with
         * @param pose the pose parameters (r,p,y,x,y,z)
         */
        void getRTmatrix(Matrix4d &RT, VectorXd &pose);

        void getTFtransform(VectorXd &x, tf::Transform &tf);

        /**
         * Sets a tf transform using a 4x4 matrix
         * @param RT using this matrix
         * @param tf setting this transform
         */
        void getTFtransform(Matrix4d &RT, tf::Transform &tf);

        void publishTF(tf::Transform &tf, const char *frame, const char *name);

    private:
        tf::TransformListener tf_listener;
        tf::TransformBroadcaster tf_broadcaster;
    };

}