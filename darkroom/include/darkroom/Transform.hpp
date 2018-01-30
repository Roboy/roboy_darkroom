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
    description: helper class for handling tf in all its facettes
*/

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

        /**
         * Calculates the quaternion and origin from pose
         * @param q Quaternion will be filled
         * @param origin 3d location will be filled
         * @param pose input (r,p,y,x,y,z)
         */
        void getPose(Quaterniond &q, Vector3d &origin, VectorXd &pose);

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